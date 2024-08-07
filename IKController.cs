using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;


// ReSharper disable once InconsistentNaming
public class IKController : MonoBehaviour
{
    [SerializeField] public Transform _targetTransform;
    public ArmJoint[] Joints;
    public float[] Angles;

    // KR10 R900
    public const float SamplingDistance = 0.7f;    //0.7
    public const float LearningRate = 5f;          //5
    public const float LossThreshold = 0.001f;      //0.01

    public float DistanceWeight = 1f;
    public float RotationWeight = 0.15f;

    private float AwakeDistance = 18f;

    // for data graph
    public List<float> DistanceLoss = new List<float>();
    public List<float> RotationLoss = new List<float>();

    public int maxDataLength = 1000;

    private void Start()
    {
        // initalizae angles for ForwardForwardKinematics
        float[] angles = new float[Joints.Length];

        for (int i = 0; i < Joints.Length; i++)
        {
            if (Joints[i]._rotationAxis == 'x')
            {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.x;
            }
            else if (Joints[i]._rotationAxis == 'y')
            {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.y;
            }
            else if (Joints[i]._rotationAxis == 'z')
            {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.z;
            }
        }

        Angles = angles;

        DistanceLoss.Clear();
        RotationLoss.Clear();
    }

    private void Update()
    {
        GameObject player = GameObject.FindGameObjectWithTag("Player");
        Vector3 playerPosition = player.transform.position;

        Vector3 globalRobotPosition = this.transform.TransformPoint(this.transform.localPosition);

        float distanceToPlayer = Vector3.Distance(globalRobotPosition, playerPosition);

        if (distanceToPlayer <= AwakeDistance)
        {
            InverseKinematics(_targetTransform.eulerAngles, _targetTransform.position, Angles);
        }
        //record data
        if (DistanceLoss.Count > maxDataLength || Input.GetKeyDown(KeyCode.D))
        {
            ExportDataToCSV();
        }
    }

    public Vector3 ForwardKinematics(float[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].RotationAxis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].StartOffset;
            prevPoint = nextPoint;
        }
        return prevPoint;
    }

    public float DistanceFromTarget(Vector3 target, float[] angles)
    {
        Vector3 point = ForwardKinematics(angles);
        return Vector3.Distance(point, target);
    }

    public float RotationFromTarget(Vector3 _targetRotation, Vector3 target, float[] angles)
    {
        Vector3 point = ForwardKinematics(angles);
        Vector3 extruderDir = point - Joints[Joints.Length - 2].transform.position;

        float EangleWithYAxis = Vector3.Angle(extruderDir, -Vector3.up);
        float targetRotation = _targetTransform.transform.eulerAngles.z;

        float ErotationDifference = EangleWithYAxis - targetRotation;

        return ErotationDifference;
    }

    public float MutiVariableLoss(Vector3 _targetRotation, Vector3 target, float[] angles)
    {
        if(DistanceLoss.Count < maxDataLength )
        {
            DistanceLoss.Add(DistanceFromTarget(target, angles));
            RotationLoss.Add(RotationFromTarget(_targetRotation, target, angles));
        }
        Debug.Log("Distance Loss£º" + DistanceFromTarget(target, angles) + "Rotation Loss: " + RotationFromTarget(_targetRotation, target, angles));
        float LossValue = DistanceWeight * DistanceFromTarget(target, angles) + RotationWeight * RotationFromTarget(_targetRotation, target, angles);
        return LossValue;
    }

    # region for data graph
    public void ExportDataToCSV()
    {
        StringBuilder csvContent = new StringBuilder();
        csvContent.AppendLine("Index,DistanceLoss,RotationLoss");

        for (int i = 0; i < DistanceLoss.Count; i++)
        {
            csvContent.AppendLine($"{i},{DistanceLoss[i]},{RotationLoss[i]}");
        }

        string fileName = "GRADIENT DESCENT_data_export.csv";
        string path = Path.Combine(Application.persistentDataPath, fileName);
        File.WriteAllText(path, csvContent.ToString());

        Debug.Log($"Data exported to {path}");
    }
    #endregion

    public float PartialGradient(Vector3 _targetRotation, Vector3 target, float[] angles, int i)
    {
        // Saves the angle,
        // it will be restored later
        float angle = angles[i];

        // Gradient using central difference : [F(x+SamplingDistance) - F(x-SamplingDistance)] / 2h
        angles[i] += SamplingDistance;
        float f_x_plus_d = MutiVariableLoss(_targetRotation, target, angles);

        angles[i] = angle - SamplingDistance;
        float f_x_minus_d = MutiVariableLoss(_targetRotation, target, angles);

        float gradient = (f_x_plus_d - f_x_minus_d) / (2 * SamplingDistance);

        // Restores
        angles[i] = angle;

        return gradient;
    }


    public void InverseKinematics(Vector3 _targetRotation, Vector3 target, float[] angles)
    {
        if (MutiVariableLoss(_targetRotation, target, angles) < LossThreshold)
            return;

        for (int i = Joints.Length - 1; i >= 0; i--)
        {
            // Gradient descent
            // Update : Solution -= LearningRate * Gradient
            float gradient = PartialGradient(_targetRotation, target, angles, i);
            angles[i] -= LearningRate * gradient;
            ApplyRotation(_targetRotation, i, target, angles);
        }

    }

    private void ApplyRotation(Vector3 _targetRotation, int i, Vector3 target, float[] angles)
    {
        if (MutiVariableLoss(_targetRotation, target, angles) < LossThreshold) return;

        switch (Joints[i]._rotationAxis)
        {
            case 'x':
                Joints[i].transform.localEulerAngles = new Vector3(angles[i], 0, 0);
                break;
            case 'y':
                Joints[i].transform.localEulerAngles = new Vector3(0, angles[i], 0);
                break;
            case 'z':
                Joints[i].transform.localEulerAngles = new Vector3(0, 0, angles[i]);
                break;
        }
    }


    void OnDrawGizmos()
    {
        if (Angles != null && _targetTransform != null)
        {
            // distance to target 
            Vector3 endEffectorPosition = Joints[Joints.Length - 1].transform.position;
            Vector3 targetPosition = _targetTransform.position;

            // Draw the main line
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(endEffectorPosition, targetPosition);

            // Draw a small sphere at the target position
            float sphereRadius = 0.05f;  // You can modify this
            Gizmos.DrawSphere(targetPosition, sphereRadius);

            // Rotation Index
            Vector3 beforeEndEffectorPosition = Joints[Joints.Length - 2].transform.position;
            Gizmos.color = Color.green;
            Gizmos.DrawLine(endEffectorPosition, beforeEndEffectorPosition);
            Gizmos.DrawSphere(endEffectorPosition, sphereRadius);
        }
    }

}