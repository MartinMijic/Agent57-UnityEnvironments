using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.IO;
public class MoveToGoal3Goals : Agent
{
    #region Variables
    [SerializeField] private Material floorStandardMaterial;
    [SerializeField] private Material finishReached;
    [SerializeField] private Material looseMaterial;
    [SerializeField] private Material goalChecked;
    [SerializeField] private Material goalUnchecked;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    Rigidbody rb;
    private float initialHeight;
    private float currDistanceToFinish;
    private float lastDistanceToFinish;

    [Header("Action Properties")]
    [SerializeField] private bool UseContinuousActions = true;

    [Header("Goal Properties")]
    [SerializeField] private List<GameObject> Goals = new List<GameObject>();
    [SerializeField] private GameObject finishPlane;
    public int NumGoals;
    public int NumGoalsNotChecked;

    [Header("Reward Properties")]
    [SerializeField] private bool HardRewardMode = false;
    #endregion

    #region Initialization and Episode Start
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        initialHeight = transform.localPosition.y;
        NumGoals = Goals.Count;
    }

    public override void OnEpisodeBegin()
    {
        resetEnvironment();             // Reset Agent and Goal positions
        NumGoalsNotChecked = NumGoals;  // Same at beginning of the episode
    }
    #endregion

    #region Observations
    public override void CollectObservations(VectorSensor sensor)
    {
        //WriteTransformString();
        if (HardRewardMode)
        {
            sensor.AddObservation(transform.localPosition);
            sensor.AddObservation(transform.localRotation);
            sensor.AddObservation(transform.InverseTransformDirection(rb.velocity));
        }
        else {
            sensor.AddObservation(transform.localPosition);
            sensor.AddObservation(transform.localRotation);
            sensor.AddObservation(transform.InverseTransformDirection(rb.velocity));
            sensor.AddObservation(NumGoalsNotChecked);
            //sensor.AddObservation(StepCount / (float)MaxStep);
        }

    }
    #endregion

    #region Actions and movement
    // Mask discrete actions
    const int k_Right = 1;
    const int k_Left = 2;

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Continuous action space
        if (UseContinuousActions)
        {
            var dirToGo = Vector3.zero;
            var rotateDir = Vector3.zero;

            rotateDir = transform.up * actions.ContinuousActions[0];

            //WriteActionString(actions.ContinuousActions[0]);

            // Clamp rotation area to +/- 90 deg (from agent's perspective) to prevent backwards movement
            if ((actions.ContinuousActions[0] > 0) && ((transform.localEulerAngles.y < 85f) || (transform.localEulerAngles.y > 270f)))
            {
                transform.Rotate(rotateDir, Time.deltaTime * 180f);
            }
            else if ((actions.ContinuousActions[0] < 0) && ((transform.localEulerAngles.y > 275f) || (transform.localEulerAngles.y < 90f)))
            {
                transform.Rotate(rotateDir, Time.deltaTime * 180f);
            }
        }

        // Discrete action space
        else
        {
            var dirToGo = Vector3.zero;
            var rotateDir = Vector3.zero;

            //WriteActionString(actions.DiscreteActions[0]);

            switch (actions.DiscreteActions[0])
            {
                case k_Right:
                    rotateDir = transform.up * 1f;
                    break;
                case k_Left:
                    rotateDir = transform.up * -1f;
                    break;
            }

            // Clamp rotation area to +/- 90 deg (from agent's perspective)
            if ((actions.DiscreteActions[0] == k_Right) && ((transform.localEulerAngles.y < 85f) || (transform.localEulerAngles.y > 270f)))
            {
                transform.Rotate(rotateDir, Time.deltaTime * 180f);
            }
            else if ((actions.DiscreteActions[0] == k_Left) && ((transform.localEulerAngles.y > 275f) || (transform.localEulerAngles.y < 90f)))
            {
                transform.Rotate(rotateDir, Time.deltaTime * 180f);
            }
        }

        // Existential penalty
        AddReward(-1f / MaxStep);
    }

    public void FixedUpdate()
    {
        // Automatic forward movement
        rb.AddForce(transform.forward * 1f * 0.3f, ForceMode.VelocityChange);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (UseContinuousActions)
        {
            ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
            continuousActions[0] = Input.GetAxisRaw("Horizontal");
        }

        else
        {
            ActionSegment<int> discreteActions = actionsOut.DiscreteActions;

            if (Input.GetAxisRaw("Horizontal") > 0)
            {
                discreteActions[0] = k_Right;
            }
            if (Input.GetAxisRaw("Horizontal") < 0)
            {
                discreteActions[0] = k_Left;
            }
        }
    }
    #endregion

    #region Triggers and Rewards
    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "finish")
        {
            // For hard RewardMode the rewards are calculated when reaching the finish line
            if (HardRewardMode)
            {
                //WriteNumGoalsString(3 - NumGoalsNotChecked);

                SetReward(1f - NumGoalsNotChecked * 0.3f); // Base Reward for reaching the finish line + penalty for every unchecked goal
                StartCoroutine(SwapGroundMaterial(finishReached, 0.5f));
                EndEpisode();
            }
            else
            {
                //WriteNumGoalsString(3 - NumGoalsNotChecked);

                SetReward(1f);
                StartCoroutine(SwapGroundMaterial(finishReached, 0.5f));
                EndEpisode();
            }

        }

        else if (other.gameObject.tag == "wall" && !((transform.localEulerAngles.y < 85f) || (transform.localEulerAngles.y > 275f)))
        {
            SetReward(-1f);
            StartCoroutine(SwapGroundMaterial(looseMaterial, 0.5f));
            EndEpisode();
        }

        else if (other.gameObject.tag == "goal")
        {
            if (HardRewardMode)
            {
                NumGoalsNotChecked--;
            }
            else
            {
                SetReward(0.3f);
                NumGoalsNotChecked--;
            }
            other.GetComponent<Renderer>().material = goalChecked;
        }

        else if (other.gameObject.tag == "pole")
        {
            SetReward(-0.1f);
            ResetAgentAfterCrash();
        }

    }
    #endregion

    #region Additional Methods
    IEnumerator SwapGroundMaterial(Material mat, float time)
    {
        floorMeshRenderer.material = mat;
        yield return new WaitForSeconds(time);
        floorMeshRenderer.material = floorStandardMaterial;
    }

    public void ResetAgentAfterCrash()
    {
        //WriteNumGoalsString(3 - NumGoalsNotChecked);

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.localPosition = new Vector3(transform.localPosition.x, transform.localPosition.y, transform.localPosition.z + 2f);
        transform.rotation = Quaternion.identity;
    }

    public void resetEnvironment()
    {
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.localPosition = new Vector3(Random.Range(-21f, 21f), initialHeight, Random.Range(-595f, -592f));
        transform.rotation = Quaternion.identity;

        // Reset the goals' positions and materials 
        foreach (GameObject goal in Goals)
        {
            goal.transform.localPosition = new Vector3(Random.Range(-23f, 13.5f), goal.transform.localPosition.y, goal.transform.localPosition.z);
            var TriggerPlane = goal.transform.Find("TriggerPlane").gameObject;
            TriggerPlane.GetComponent<Renderer>().material = goalUnchecked;
        }
    }
    #endregion

    public void WriteTransformString()
    {
        string path = Application.persistentDataPath + "/skiing_transforms_log.csv";

        //Write some text to the test.txt file

        StreamWriter writer = new StreamWriter(path, true);
        var LineToWrite = transform.localPosition.x + ";" + transform.localPosition.y + ";" + transform.localPosition.z;
        writer.WriteLine(LineToWrite);
        writer.Close();
    }

    public void WriteActionString(float rotation)
    {
        string path = Application.persistentDataPath + "/skiing_actions_log.csv";

        StreamWriter writer = new StreamWriter(path, true);
        var LineToWrite = rotation;
        writer.WriteLine(LineToWrite);
        writer.Close();
    }

    public void WriteNumGoalsString(int numGoals)
    {
        string path = Application.persistentDataPath + "/skiing_goals_log.csv";

        StreamWriter writer = new StreamWriter(path, true);
        var LineToWrite = numGoals;
        writer.WriteLine(LineToWrite);
        writer.Close();
    }
}