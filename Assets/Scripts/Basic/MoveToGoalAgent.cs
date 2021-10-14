using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class MoveToGoalAgent : Agent
{
    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(Random.Range(-2f,1f), 0, Random.Range(-2f,2f));
        //transform.localPosition = new Vector3(4.5f, -0.21f, -0.03f);
        //targetTransform.localPosition = new Vector3(Random.Range(10f, 12.5f), 0, Random.Range(-2f, 2f));
    }

    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material looseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;
    [SerializeField] private bool UseContinuousActions = false;
    [SerializeField] private float MovSpeedDiscrete = 0.08f;
    [SerializeField] private float MovSpeedContinuous = 1f;
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTransform.localPosition);
        sensor.AddObservation(Vector3.Distance(transform.localPosition, targetTransform.localPosition));
        sensor.AddObservation(StepCount / (float)MaxStep);
    }

    // Mask discrete actions
    const int k_Forward = 0;  // do nothing!
    const int k_Backwards = 1;
    const int k_Right = 2;
    const int k_Left = 3;
    const int k_NoAction = 4;

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (UseContinuousActions)
        {
            float moveX = actions.ContinuousActions[0];
            float moveZ = actions.ContinuousActions[1];
            transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * MovSpeedContinuous;

        }

        else {
            switch (actions.DiscreteActions[0]) {
                case k_Forward:
                    transform.localPosition += new Vector3(MovSpeedDiscrete, 0, 0) * Time.deltaTime; //Move x-Positive
                    break;
                case k_Backwards:
                    transform.localPosition += new Vector3(-MovSpeedDiscrete, 0, 0) * Time.deltaTime; //Move x-Negative
                    break;
                case k_Right:
                    transform.localPosition += new Vector3(0, 0, MovSpeedDiscrete) * Time.deltaTime; //Move z-Positive
                    break;
                case k_Left:
                    transform.localPosition += new Vector3(0, 0, -MovSpeedDiscrete) * Time.deltaTime; //Move z-Negative
                    break;
                case k_NoAction:
                    transform.localPosition += new Vector3(0, 0, 0); //Don't move
                    break;
            }
        }
        AddReward(-1f / MaxStep);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (UseContinuousActions)
        {
            ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
            continuousActions[0] = Input.GetAxisRaw("Horizontal");
            continuousActions[1] = Input.GetAxisRaw("Vertical");
        }
        else {
            ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
            discreteActions[0] = k_NoAction;

            if (Input.GetAxisRaw("Horizontal") > 0)
            {
                discreteActions[0] = k_Forward;
            }
            if (Input.GetAxisRaw("Horizontal") < 0)
            {
                discreteActions[0] = k_Backwards;
            }
            if (Input.GetAxisRaw("Vertical") > 0)
            {
                discreteActions[0] = k_Right;
            }
            if (Input.GetAxisRaw("Vertical") < 0)
            {
                discreteActions[0] = k_Left;
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "goal") {
            SetReward(1f);
            floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }

        else if (other.gameObject.tag == "Border")
        {
            SetReward(-0.1f);
            floorMeshRenderer.material = looseMaterial;
            EndEpisode();
        }
    }
}
