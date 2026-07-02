using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using Unity.XR.PXR;

public class PicoLeUr5HandStreamer : MonoBehaviour
{
    [Header("Computer TCP Server")]
    public string computerHost = "192.168.1.50";
    public int computerPort = 8000;
    public float streamHz = 30.0f;

    [Header("Runtime")]
    public bool connectOnStart = true;
    public bool verbose = false;

    private TcpClient client;
    private NetworkStream stream;
    private readonly UTF8Encoding utf8 = new UTF8Encoding(false);
    private float nextSendTime;

    private static readonly HandJoint[] MediaPipe21FromPico =
    {
        HandJoint.JointWrist,
        HandJoint.JointThumbMetacarpal,
        HandJoint.JointThumbProximal,
        HandJoint.JointThumbDistal,
        HandJoint.JointThumbTip,
        HandJoint.JointIndexProximal,
        HandJoint.JointIndexIntermediate,
        HandJoint.JointIndexDistal,
        HandJoint.JointIndexTip,
        HandJoint.JointMiddleProximal,
        HandJoint.JointMiddleIntermediate,
        HandJoint.JointMiddleDistal,
        HandJoint.JointMiddleTip,
        HandJoint.JointRingProximal,
        HandJoint.JointRingIntermediate,
        HandJoint.JointRingDistal,
        HandJoint.JointRingTip,
        HandJoint.JointLittleProximal,
        HandJoint.JointLittleIntermediate,
        HandJoint.JointLittleDistal,
        HandJoint.JointLittleTip,
    };

    private void Start()
    {
        if (connectOnStart)
        {
            Connect();
        }
    }

    private void Update()
    {
        if (Time.unscaledTime < nextSendTime)
        {
            return;
        }

        nextSendTime = Time.unscaledTime + 1.0f / Mathf.Max(streamHz, 1.0f);

        if (!IsConnected())
        {
            Connect();
            return;
        }

        SendRightHandFrame();
    }

    public void Connect()
    {
        Disconnect();

        try
        {
            client = new TcpClient();
            client.NoDelay = true;
            client.Connect(computerHost, computerPort);
            stream = client.GetStream();

            if (verbose)
            {
                Debug.Log($"PICO LeUr5 streamer connected to {computerHost}:{computerPort}");
            }
        }
        catch (Exception ex)
        {
            if (verbose)
            {
                Debug.LogWarning($"PICO LeUr5 streamer connect failed: {ex.Message}");
            }
            Disconnect();
        }
    }

    public void Disconnect()
    {
        try
        {
            stream?.Close();
            client?.Close();
        }
        catch
        {
            // Ignore shutdown errors.
        }

        stream = null;
        client = null;
    }

    private bool IsConnected()
    {
        return client != null && client.Connected && stream != null;
    }

    private void SendRightHandFrame()
    {
        if (!PXR_HandTracking.GetSettingState())
        {
            if (verbose)
            {
                Debug.LogWarning("PICO hand tracking setting is disabled");
            }
            return;
        }

        var locations = new HandJointLocations();
        if (!PXR_HandTracking.GetJointLocations(HandType.HandRight, ref locations))
        {
            return;
        }

        if (locations.isActive == 0 || locations.jointLocations == null)
        {
            return;
        }

        if (locations.jointLocations.Length <= (int)HandJoint.JointLittleTip)
        {
            if (verbose)
            {
                Debug.LogWarning($"PICO joint array too short: {locations.jointLocations.Length}");
            }
            return;
        }

        var aimState = new HandAimState();
        PXR_HandTracking.GetAimState(HandType.HandRight, ref aimState);

        HandJointLocation wristLocation = locations.jointLocations[(int)HandJoint.JointWrist];
        Vector3 wristPosition = ConvertUnityPositionForLeFranX(ToUnityVector3(wristLocation.pose.Position));
        Quaternion wristRotation = ConvertUnityRotationForLeFranX(ToUnityQuaternion(wristLocation.pose.Orientation));
        string fistState = GetFistState(aimState);

        string wristMessage =
            "Right wrist:, " +
            F(wristPosition.x) + ", " +
            F(wristPosition.y) + ", " +
            F(wristPosition.z) + ", " +
            F(wristRotation.x) + ", " +
            F(wristRotation.y) + ", " +
            F(wristRotation.z) + ", " +
            F(wristRotation.w) + ", leftFist: " +
            fistState + "\n";

        var landmarkBuilder = new StringBuilder(1024);
        landmarkBuilder.Append("Right landmarks: ");

        for (int i = 0; i < MediaPipe21FromPico.Length; i++)
        {
            HandJoint joint = MediaPipe21FromPico[i];
            HandJointLocation jointLocation = locations.jointLocations[(int)joint];
            Vector3 p = ConvertUnityPositionForLeFranX(ToUnityVector3(jointLocation.pose.Position));

            if (i > 0)
            {
                landmarkBuilder.Append(",");
            }

            landmarkBuilder.Append(F(p.x));
            landmarkBuilder.Append(",");
            landmarkBuilder.Append(F(p.y));
            landmarkBuilder.Append(",");
            landmarkBuilder.Append(F(p.z));
        }

        landmarkBuilder.Append("\n");

        SendString(wristMessage);
        SendString(landmarkBuilder.ToString());
    }

    private void SendString(string message)
    {
        if (!IsConnected())
        {
            return;
        }

        try
        {
            byte[] bytes = utf8.GetBytes(message);
            stream.Write(bytes, 0, bytes.Length);
        }
        catch (Exception ex)
        {
            if (verbose)
            {
                Debug.LogWarning($"PICO LeUr5 streamer send failed: {ex.Message}");
            }
            Disconnect();
        }
    }

    private static string GetFistState(HandAimState aimState)
    {
        bool indexPinching = (aimState.aimStatus & (ulong)HandAimStatus.AimIndexPinching) != 0;
        bool middlePinching = (aimState.aimStatus & (ulong)HandAimStatus.AimMiddlePinching) != 0;
        bool ringPinching = (aimState.aimStatus & (ulong)HandAimStatus.AimRingPinching) != 0;
        bool littlePinching = (aimState.aimStatus & (ulong)HandAimStatus.AimLittlePinching) != 0;

        return indexPinching && middlePinching && ringPinching && littlePinching ? "fist" : "open";
    }

    private static Vector3 ConvertUnityPositionForLeFranX(Vector3 position)
    {
        // Keep Unity/PICO world axes here. The Python UR5e teleoperator applies
        // the VR-to-robot transform. If your PICO app uses a different origin or
        // handedness, adjust this method first.
        return position;
    }

    private static Quaternion ConvertUnityRotationForLeFranX(Quaternion rotation)
    {
        return rotation;
    }

    private static Vector3 ToUnityVector3(Vector3f vector)
    {
        return new Vector3(vector.x, vector.y, vector.z);
    }

    private static Quaternion ToUnityQuaternion(Quatf quat)
    {
        return new Quaternion(quat.x, quat.y, quat.z, quat.w);
    }

    private static string F(float value)
    {
        return value.ToString("G9", CultureInfo.InvariantCulture);
    }

    private void OnDestroy()
    {
        Disconnect();
    }
}
