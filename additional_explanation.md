これらのプログラムは、"zx200_ros2(https://github.com/pwri-opera/zx200_ros2)"のナビゲーションに直線追従用のプログラムを追加したプログラムである。

### 直線追従ナビゲーションの追加部分に関する説明

|branch|用途 |使用法 |
| ------------ | -------- | ---- |
|feature/add_nabigation| GNSSでの直進追従ナビゲーション用 | ゴールはtopicかRVIZより与えること(TMS非連携)|
|feature/add_nabigation_for_AR|Arucoマーカーでの直進追従ナビゲーション用 | ゴールはtopicかRVIZより与えること(TMS非連携) |
|feature/add_nabigation_for_TMS| GNSS, Arucoマーカーでの直進追従ナビゲーション用| TMSと連携。TMSでfollow_straightを呼び出すと直線追従。ゴールをtopicかRVIZより与えた場合は、navigate_to_poseのデフォルトのBT(zx200_navigation/params/zx200_navigate_to_pose_w_replanning_and_recovery.xml)が読み出される。|



### ナビゲーションの起動用コマンド(GNSS使用)
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch zx200_bringup remote_navigation.launch.py
```

### ナビゲーションの起動用コマンド(ARマーカー使用)
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch zx200_bringup remote_navigation_for_aruco.launch.py 
```



### シミュレータ用の追加部分

プログラムは、実機に即したものとなっている。そのためOperaSimでナビゲーションを行う際には、OperaSimにGNSSの位置の出力を行う次のC#プログラムを追加すること。

```
﻿using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using System.Collections;

public class PosePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_pose";
    ROSClockPublisher ROSClockPublisher;
    public GameObject ClockObject;
    public float OffsetX;
    public float OffsetY;
    public float OffsetZ;

    public float OffsetRotY;


    public float noiseStdDevPoint = 0.1f;   // ノイズの強さ
    public float noiseStdDevRot = 0.1f;   // ノイズの強さ
    public float updateInterval = 0.1f; // 更新間隔（秒）

    private Vector3 basePosition;
    private Vector4 currentNoise;

    public Quaternion MachineRotation;

    public bool Noise;
    public bool RotNoise;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);

        ClockObject = GameObject.Find("WorldClock");
        ROSClockPublisher = ClockObject.GetComponent<ROSClockPublisher>();
        //////
        basePosition = transform.position;

        // コルーチン開始
        StartCoroutine(NoiseRoutine());
    }

    void Update()
    {
        PoseStampedMsg poseMsg = new PoseStampedMsg();

        // Header
        poseMsg.header = new HeaderMsg();
        poseMsg.header.stamp = new TimeMsg(
        (int)System.DateTimeOffset.Now.ToUnixTimeSeconds(),
        (uint)(System.DateTime.Now.Millisecond * 1000000)
        );
        //
        poseMsg.header.stamp = new TimeMsg(
            (int)ROSClockPublisher.second,
            (uint)ROSClockPublisher.nanosecond
            );
        //
        poseMsg.header.frame_id = "";

        poseMsg.pose.position.x = transform.position.x + OffsetX;
        poseMsg.pose.position.y = transform.position.z + OffsetY;
        poseMsg.pose.position.z = transform.position.y + OffsetZ;

        if (Noise == true)
        {
            // Position
            poseMsg.pose.position.x = transform.position.x + OffsetX + currentNoise.x;
            poseMsg.pose.position.y = transform.position.z + OffsetY + currentNoise.y;
            poseMsg.pose.position.z = transform.position.y + OffsetZ + currentNoise.z;
        }

        // Rotation
        MachineRotation = Quaternion.Euler(transform.rotation.eulerAngles - new Vector3(0.0f, OffsetRotY, 0.0f));
        if (RotNoise == true)
        {
            MachineRotation = Quaternion.Euler(transform.rotation.eulerAngles - new Vector3(0.0f, OffsetRotY+ currentNoise.w, 0.0f));
        }
        poseMsg.pose.orientation.x = MachineRotation.z;
        poseMsg.pose.orientation.y = -MachineRotation.x;
        poseMsg.pose.orientation.z = MachineRotation.y;
        poseMsg.pose.orientation.w = -MachineRotation.w;

        ros.Publish(topicName, poseMsg);
    }

    IEnumerator NoiseRoutine()
    {
        while (true)
        {
            currentNoise = new Vector4(
                Gaussian(0f, noiseStdDevPoint),
                Gaussian(0f, noiseStdDevPoint),
                Gaussian(0f, noiseStdDevPoint),
                Gaussian(0f, noiseStdDevRot)
            );

            yield return new WaitForSeconds(updateInterval); // 例: 0.1秒（10Hz）
        }
    }

    float Gaussian(float mean, float stdDev)
    {
        float u1 = 1.0f - Random.value;
        float u2 = 1.0f - Random.value;

        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                              Mathf.Sin(2.0f * Mathf.PI * u2);

        return mean + stdDev * randStdNormal;
    }
}
```

---
Arucoマーカーでのシミュレーションの場合は次のC#プログラムをOperaSimに追加すること。
```
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class ArucoPublisher : MonoBehaviour
{
    ROSConnection ros;
    ROSClockPublisher ROSClockPublisher;
    public GameObject ClockObject;
    public float OffsetX;
    public float OffsetY;
    public float OffsetZ;
    public float OffsetRotY;
    public Quaternion MachineRotation;

    public string topicName = "/zx200/odometry/global/aruco";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);

        ClockObject = GameObject.Find("WorldClock");
        ROSClockPublisher = ClockObject.GetComponent<ROSClockPublisher>();
    }

    void Update()
    {
        var msg = new OdometryMsg();

        // Header
        msg.header = new HeaderMsg
        {
            stamp = new TimeMsg
            {
                sec = (int)ROSClockPublisher.second,
                nanosec = (uint)ROSClockPublisher.nanosecond
            },
            frame_id = "map"
        };

        msg.child_frame_id = "base_link";

        // Pose
        MachineRotation = Quaternion.Euler(transform.rotation.eulerAngles - new Vector3(0.0f, OffsetRotY, 0.0f));
        
        msg.pose = new PoseWithCovarianceMsg();
        msg.pose.pose = new PoseMsg
        {
            position = new PointMsg
            {
                x = transform.position.x + OffsetX,
                y = transform.position.z + OffsetY,
                z = transform.position.y + OffsetZ
            },
            orientation = new QuaternionMsg
            {
                x = MachineRotation.z,
                y = -MachineRotation.x,
                z = MachineRotation.y,
                w = -MachineRotation.w
            }
        };

        // covariance (36要素)
        msg.pose.covariance = new double[36];

        // Twist
        msg.twist = new TwistWithCovarianceMsg();
        msg.twist.twist = new TwistMsg
        {
            linear = new Vector3Msg
            {
                x = 0.0,
                y = 0.0,
                z = 0.0
            },
            angular = new Vector3Msg
            {
                x = 0.0,
                y = 0.0,
                z = 0.0
            }
        };

        msg.twist.covariance = new double[36];

        ros.Publish(topicName, msg);
    }
}
```