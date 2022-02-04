using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
// 19 Discrete Actions
// 
public class WormAgent : Agent
{
    const float m_MaxWalkingSpeed = 10; //The max walking speed

    [Header("Target Prefabs")] public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    private Transform m_Target; //Target the agent will walk towards during training.

    [Header("Body Parts")] public Transform bodySegment0;
    public Transform bodySegment1;
    public Transform bodySegment2;
    public Transform bodySegment3;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    private Vector3 m_StartingPos; //starting position of the agent
    
    [Header("Action Properties")]
    [SerializeField] private bool UseContinuousActions = true;

    /// BS*Num*_*Axis*_*PositiveOrNegative* == BodySegment*Number*_*AxisToIncrement*_*IncrementPosOrNeg*
    public float incrementationStrength = 0.25f;

    // Joint Rotations
    private float BS0_x;
    private float BS0_y;
    private float BS1_x;
    private float BS1_y;
    private float BS2_x;
    private float BS2_y;

    // Joint Strength
    private float BS0_JS;
    private float BS1_JS;
    private float BS2_JS;

    public override void Initialize()
    {
        SpawnTarget(TargetPrefab, transform.position); //spawn target

        m_StartingPos = bodySegment0.position;
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        UpdateOrientationObjects();

        //Setup each body part
        m_JdController.SetupBodyPart(bodySegment0);
        m_JdController.SetupBodyPart(bodySegment1);
        m_JdController.SetupBodyPart(bodySegment2);
        m_JdController.SetupBodyPart(bodySegment3);
    }


    /// <summary>
    /// Spawns a target prefab at pos
    /// </summary>
    /// <param name="prefab"></param>
    /// <param name="pos"></param>
    void SpawnTarget(Transform prefab, Vector3 pos)
    {
        m_Target = Instantiate(prefab, pos, Quaternion.identity, transform.parent);
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        // Joint Rotations
        BS0_x = 0f;
        BS0_y = 0f;
        BS1_x = 0f;
        BS1_y = 0f;
        BS2_x = 0f;
        BS2_y = 0f;
        // Joint strength
        BS0_JS = 0f;
        BS1_JS = 0f;
        BS2_JS = 0f;

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        bodySegment0.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();
    }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround ? 1 : 0); // Whether the bp touching the ground

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));


        if (bp.rb.transform != bodySegment0)
        {
            //Get position relative to hips in the context of our orientation cube's space
            sensor.AddObservation(
                m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - bodySegment0.position));
            sensor.AddObservation(bp.rb.transform.localRotation);
        }

        if (bp.joint)
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        RaycastHit hit;
        float maxDist = 10;
        if (Physics.Raycast(bodySegment0.position, Vector3.down, out hit, maxDist))
        {
            sensor.AddObservation(hit.distance / maxDist);
        }
        else
            sensor.AddObservation(1);

        var cubeForward = m_OrientationCube.transform.forward;
        var velGoal = cubeForward * m_MaxWalkingSpeed;
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        sensor.AddObservation(Quaternion.Angle(m_OrientationCube.transform.rotation,
                                  m_JdController.bodyPartsDict[bodySegment0].rb.rotation) / 180);
        sensor.AddObservation(Quaternion.FromToRotation(bodySegment0.forward, cubeForward));

        //Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.transform.position));

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }

        //sensor.AddObservation(StepCount / (float)MaxStep);
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        AddReward(1f);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var i = -1;
        if (UseContinuousActions)
        {
            var continuousActions = actionBuffers.ContinuousActions;
            // Pick a new target joint rotation
            bpDict[bodySegment0].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
            bpDict[bodySegment1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
            bpDict[bodySegment2].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

            // Update joint strength
            bpDict[bodySegment0].SetJointStrength(continuousActions[++i]);
            bpDict[bodySegment1].SetJointStrength(continuousActions[++i]);
            bpDict[bodySegment2].SetJointStrength(continuousActions[++i]);
        }

        else
        {
            var discreteActions = actionBuffers.DiscreteActions[0];
            // Action "0" means no action ("do nothing")
            switch (discreteActions)
            {
                #region Joint Rotation: BodySegment 0
                case 1:
                    // Clamp to max 1
                    if (BS0_x < 1f) {
                        BS0_x = BS0_x + incrementationStrength;
                        bpDict[bodySegment0].SetJointTargetRotation(BS0_x, BS0_y, 0); // Rise X
                    }
                    break;
                case 2:
                    // Clamp to min -1
                    if (BS0_x > -1f)
                    {
                        BS0_x = BS0_x - incrementationStrength;
                        bpDict[bodySegment0].SetJointTargetRotation(BS0_x, BS0_y, 0); // Lower X
                    }
                    break;
                case 3:
                    if (BS0_y < 1f)
                    {
                        BS0_y = BS0_y + incrementationStrength;
                        bpDict[bodySegment0].SetJointTargetRotation(BS0_x, BS0_y, 0); // Rise Y
                    }
                    break;
                case 4:
                    if (BS0_y > -1f)
                    {
                        BS0_y = BS0_y - incrementationStrength;
                        bpDict[bodySegment0].SetJointTargetRotation(BS0_x, BS0_y, 0); // Lower Y
                    }
                    break;
                #endregion

                #region Joint Rotation: BodySegment 1
                case 5:
                    // Clamp to max 1
                    if (BS1_x < 1f)
                    {
                        BS1_x = BS1_x + incrementationStrength;
                        bpDict[bodySegment1].SetJointTargetRotation(BS1_x, BS1_y, 0); // Rise X
                    }
                    break;
                case 6:
                    // Clamp to min -1
                    if (BS1_x > -1f)
                    {
                        BS1_x = BS1_x - incrementationStrength;
                        bpDict[bodySegment1].SetJointTargetRotation(BS1_x, BS1_y, 0); // Lower X
                    }
                    break;
                case 7:
                    if (BS1_y < 1f)
                    {
                        BS1_y = BS1_y + incrementationStrength;
                        bpDict[bodySegment1].SetJointTargetRotation(BS1_x, BS1_y, 0); // Rise Y
                    }
                    break;
                case 8:
                    if (BS1_y > -1f)
                    {
                        BS1_y = BS1_y - incrementationStrength;
                        bpDict[bodySegment1].SetJointTargetRotation(BS1_x, BS1_y, 0); // Lower Y
                    }
                    break;
                #endregion

                #region Joint Rotation: BodySegment 2
                case 9:
                    // Clamp to max 1
                    if (BS2_x < 1f)
                    {
                        BS2_x = BS2_x + incrementationStrength;
                        bpDict[bodySegment2].SetJointTargetRotation(BS2_x, BS2_y, 0); // Rise X
                    }
                    break;
                case 10:
                    // Clamp to min -1
                    if (BS2_x > -1f)
                    {
                        BS2_x = BS2_x - incrementationStrength;
                        bpDict[bodySegment2].SetJointTargetRotation(BS2_x, BS2_y, 0); // Lower X
                    }
                    break;
                case 11:
                    if (BS2_y < 1f)
                    {
                        BS2_y = BS2_y + incrementationStrength;
                        bpDict[bodySegment2].SetJointTargetRotation(BS2_x, BS2_y, 0); // Rise Y
                    }
                    break;
                case 12:
                    if (BS2_y > -1f)
                    {
                        BS2_y = BS2_y - incrementationStrength;
                        bpDict[bodySegment2].SetJointTargetRotation(BS2_x, BS2_y, 0); // Lower Y
                    }
                    break;
                #endregion

                #region Joint Strength: BodySegment 0
                case 13:
                    BS0_JS = BS0_JS + incrementationStrength;
                    if (BS0_JS < 1f)
                    {
                        bpDict[bodySegment0].SetJointStrength(BS0_JS); // Rise
                    }
                    break;
                case 14:
                    BS0_JS = BS0_JS - incrementationStrength;
                    if (BS0_JS > -1f)
                    {
                        bpDict[bodySegment0].SetJointStrength(BS0_JS); // Lower
                    }
                    break;
                #endregion

                #region Joint Strength: BodySegment 1
                case 15:
                    BS1_JS = BS1_JS + incrementationStrength;
                    if (BS1_JS < 1f)
                    {
                        bpDict[bodySegment1].SetJointStrength(BS1_JS);
                    }
                    break;
                case 16:
                    BS1_JS = BS1_JS - incrementationStrength;
                    if (BS1_JS > -1f)
                    {
                        bpDict[bodySegment1].SetJointStrength(BS1_JS);
                    }
                    break;
                #endregion

                #region Joint Strength: BodySegment 2
                case 17:
                    BS2_JS = BS2_JS + incrementationStrength;
                    if (BS2_JS < 1f)
                    {
                        bpDict[bodySegment2].SetJointStrength(BS2_JS);
                    }
                    break;
                case 18:
                    BS2_JS = BS2_JS - incrementationStrength;
                    if (BS2_JS > -1f)
                    {
                        bpDict[bodySegment2].SetJointStrength(BS2_JS);
                    }
                    break;
                #endregion
            }
        }

        //Reset if Worm fell through floor;
        if (bodySegment0.position.y < m_StartingPos.y - 2)
        {
            EndEpisode();
        }
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        var velReward =
            GetMatchingVelocityReward(m_OrientationCube.transform.forward * m_MaxWalkingSpeed,
                m_JdController.bodyPartsDict[bodySegment0].rb.velocity);

        //Angle of the rotation delta between cube and body.
        //This will range from (0, 180)
        var rotAngle = Quaternion.Angle(m_OrientationCube.transform.rotation,
            m_JdController.bodyPartsDict[bodySegment0].rb.rotation);

        //The reward for facing the target
        var facingRew = 0f;
        //If we are within 30 degrees of facing the target
        if (rotAngle < 30)
        {
            //Set normalized facingReward
            //Facing the target perfectly yields a reward of 1
            facingRew = 1 - (rotAngle / 180);
        }

        //Add the product of these two rewards
        AddReward(velReward * facingRew);
    }

    /// <summary>
    /// Normalized value of the difference in actual speed vs goal walking speed.
    /// </summary>
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, m_MaxWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / m_MaxWalkingSpeed, 2), 2);
    }

    /// <summary>
    /// Update OrientationCube and DirectionIndicator
    /// </summary>
    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(bodySegment0, m_Target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }
}
