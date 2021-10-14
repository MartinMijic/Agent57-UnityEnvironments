using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/* ----------------------------------------------------CODEYARD-----------------------------------------------------
 * ------------------------------------------(Contains old code fragments)------------------------------------------
 
 * Base Parameter
 *     //[SerializeField] private Material looseMaterial;    //Not needed

 * OnEpisodeBegin
*   SpawnObjects();
    transform.localPosition = new Vector3(Random.Range(-2f, 1f), 0, Random.Range(-2f, 2f));
    targetTransform.localPosition = new Vector3(Random.Range(10f, 12.5f), 0, Random.Range(-2f, 2f));
    floorMeshRenderer.material = floorStandardMaterial;
 
 * OnActionReceived
    transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * MovSpeedContinuous;

    transform.localPosition += new Vector3(MovSpeedDiscrete, 0, 0) * Time.deltaTime; //Move x-Positive
    transform.localPosition += new Vector3(-MovSpeedDiscrete, 0, 0) * Time.deltaTime; //Move x-Negative
    transform.localPosition += new Vector3(0, 0, MovSpeedDiscrete) * Time.deltaTime; //Move z-Positive
    transform.localPosition += new Vector3(0, 0, -MovSpeedDiscrete) * Time.deltaTime; //Move z-Negative
    transform.localPosition += new Vector3(0, 0, 0); //Don't move
    
    case k_Backwards:
    dirToGo = transform.forward * -1f;
    break;  

    const int k_NoAction = 4;
    case k_NoAction:
    break;

 * OnTriggerEnter
    floorMeshRenderer.material = finishReached;
    floorMeshRenderer.material = looseMaterial;
    else if (other.gameObject.tag == "pole")
    {
        SetReward(-1f);
        StartCoroutine(GoalScoredSwapGroundMaterial(looseMaterial, 0.5f));
        EndEpisode();
    }

    
    if (Input.GetAxisRaw("Vertical") < 0)
    {
        discreteActions[0] = k_Backwards;
    }
    
*----------------------------------------------------------------------------------------------------------------    
 */

public class MoveToGoal : Agent
{
    #region Variables
    //[SerializeField] private Transform targetTransform;
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

    [Header("Movement Properties")]
    [SerializeField] private float MovSpeedDiscrete = 0.3f;
    [SerializeField] private float MovSpeedContinuous = 0.3f;
    [SerializeField] private bool AutoForwardMovement = true;
    [SerializeField] private bool ClampRotation = true;
    private bool AutoForwardWasActive = true;   // For collision with the poles

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
        AutoForwardWasActive = AutoForwardMovement;
    }
    #endregion

    #region Observations
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(transform.localRotation);
        sensor.AddObservation(transform.InverseTransformDirection(rb.velocity));
        sensor.AddObservation(NumGoalsNotChecked);
        sensor.AddObservation(StepCount / (float)MaxStep);
    }
    #endregion

    #region Actions and movement
    // Mask discrete actions
    const int k_Right = 1;
    const int k_Left = 2;
    const int k_Forward = 3;

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Existential penalty
        AddReward(-1f / MaxStep);

        // Continuous action space
        if (UseContinuousActions)
        {
            var dirToGo = Vector3.zero;
            var rotateDir = Vector3.zero;

            rotateDir = transform.up * actions.ContinuousActions[0];

            // Clamp rotation area to +/- 90 deg (from agent's perspective) to prevent backwards movement
            if (ClampRotation)
            {
                if ((actions.ContinuousActions[0] > 0) && ((transform.localEulerAngles.y < 85f) || (transform.localEulerAngles.y > 270f)))
                {
                    transform.Rotate(rotateDir, Time.deltaTime * 200f);
                }
                else if ((actions.ContinuousActions[0] < 0) && ((transform.localEulerAngles.y > 275f) || (transform.localEulerAngles.y < 90f)))
                {
                    transform.Rotate(rotateDir, Time.deltaTime * 200f);
                }
            }
            else {
                transform.Rotate(rotateDir, Time.deltaTime * 200f);
            }

            if (!AutoForwardMovement)
            {
                Debug.Log("Auto Forward Movement deactivated!");
                dirToGo = transform.forward * actions.ContinuousActions[1];
                rb.AddForce(dirToGo * MovSpeedContinuous, ForceMode.VelocityChange);
            }
        }

        // Discrete action space
        else
        {
            var dirToGo = Vector3.zero;
            var rotateDir = Vector3.zero;

            switch (actions.DiscreteActions[0])
            {
                case k_Right:
                    rotateDir = transform.up * 1f;
                    break;
                case k_Left:
                    rotateDir = transform.up * -1f;
                    break;
                case k_Forward:
                    dirToGo = transform.forward * 1f;
                    break;
            }

            // Clamp rotation area to +/- 90 deg (from agent's perspective)
            if (ClampRotation)
            {
                if ((actions.DiscreteActions[0] == k_Right) && ((transform.localEulerAngles.y < 85f) || (transform.localEulerAngles.y > 270f)))
                {
                    transform.Rotate(rotateDir, Time.deltaTime * 200f);
                }
                else if ((actions.DiscreteActions[0] == k_Left) && ((transform.localEulerAngles.y > 275f) || (transform.localEulerAngles.y < 90f)))
                {
                    transform.Rotate(rotateDir, Time.deltaTime * 200f);
                }
            }
            else
            {
                transform.Rotate(rotateDir, Time.deltaTime * 200f);
            }

            if (!AutoForwardMovement)
            {
                rb.AddForce(dirToGo * MovSpeedDiscrete, ForceMode.VelocityChange);
            }
        }
    }

    public void FixedUpdate()
    {
        // Automatic forward movement
        if (AutoForwardMovement)
        {
            AutoForwardWasActive = true;
            rb.AddForce(transform.forward * 1f * 0.3f, ForceMode.VelocityChange);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (UseContinuousActions)
        {
            ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
            continuousActions[0] = Input.GetAxisRaw("Horizontal");

            if (!AutoForwardMovement)
            {
                continuousActions[1] = Input.GetAxisRaw("Vertical");
                
                // Penalty for moving backwards
                if (continuousActions[1] < 0) 
                {
                    AddReward(-1f / MaxStep);
                }
            }
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

            if(!AutoForwardMovement)
            {
                if (Input.GetAxisRaw("Vertical") > 0)
                {
                    discreteActions[0] = k_Forward;
                }
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
                if (NumGoalsNotChecked != 0)
                {
                    var NumGoalsChecked = NumGoals - NumGoalsNotChecked;
                    SetReward(1f + NumGoalsChecked * 0.1f - NumGoalsNotChecked * 0.1f); // Base Reward for reaching the finish line + Reward for every checked goal + penalty for every unchecked goal
                }
                StartCoroutine(SwapGroundMaterial(finishReached, 0.5f));
                EndEpisode();
            }
            else
            {
                AddReward(1f);
                if (NumGoalsNotChecked != 0)
                {
                    AddReward(-0.1f * NumGoalsNotChecked); // Penalty for remaining goals after reaching the finish line
                }
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
                AddReward(0.1f);
                NumGoalsNotChecked--;
            }
            other.GetComponent<Renderer>().material = goalChecked;
        }

        else if (other.gameObject.tag == "pole")
        {
            AddReward(-0.1f);
            StartCoroutine(ResetAgentAfterCrash(1f));
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

    IEnumerator ResetAgentAfterCrash(float time)
    {
        rb.velocity = Vector3.zero;

        // Check if AutoForwardMode was active
        CheckAutoMovement(AutoForwardWasActive);

        //transform.position = new Vector3(transform.position.x, transform.position.y, transform.position.z - 0.75f);
        yield return new WaitForSeconds(time);
        
        transform.position = new Vector3(transform.position.x, transform.position.y, transform.position.z+2f);
        transform.rotation = Quaternion.identity;

        // Reactivate AutoForwardMovement if AutoForwardMode was active priorly
        CheckAutoMovement(AutoForwardWasActive);
    }

    public void CheckAutoMovement(bool AutoForwardActivity) {
        if (AutoForwardActivity && (AutoForwardMovement==false)) {
            AutoForwardMovement = true;
        }

        else if (AutoForwardActivity && (AutoForwardMovement==true)) {
            AutoForwardMovement = false;
        }
    }

    public void resetEnvironment() {
        // Deactivate AutoMovement temporarily and reset Agent's position, rotation and velocity
        CheckAutoMovement(AutoForwardWasActive);

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.localPosition = new Vector3(Random.Range(-21f, 21f), initialHeight, Random.Range(-599f, -587f));
        transform.rotation = Quaternion.identity;

        CheckAutoMovement(AutoForwardWasActive);

        // Reset the goals' positions and materials 
        foreach (GameObject goal in Goals){
            goal.transform.localPosition = new Vector3(Random.Range(-23f, 13.5f), goal.transform.localPosition.y, goal.transform.localPosition.z);
            var TriggerPlane = goal.transform.Find("TriggerPlane").gameObject;
            TriggerPlane.GetComponent<Renderer>().material = goalUnchecked;
        }
    }
    #endregion
}