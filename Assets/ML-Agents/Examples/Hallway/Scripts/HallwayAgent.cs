using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.IO;
public class HallwayAgent : Agent
{
    public GameObject ground;
    public GameObject area;
    public GameObject symbolOGoal;
    public GameObject symbolXGoal;
    public GameObject symbolO;
    public GameObject symbolX;
    public bool useVectorObs;
    Rigidbody m_AgentRb;
    Material m_GroundMaterial;
    Renderer m_GroundRenderer;
    HallwaySettings m_HallwaySettings;
    int m_Selection;
    StatsRecorder m_statsRecorder;

    [Header("Action Properties")]
    [SerializeField] private bool UseContinuousActions = true;

    public override void Initialize()
    {
        m_HallwaySettings = FindObjectOfType<HallwaySettings>();
        m_AgentRb = GetComponent<Rigidbody>();
        m_GroundRenderer = ground.GetComponent<Renderer>();
        m_GroundMaterial = m_GroundRenderer.material;
        //m_statsRecorder = Academy.Instance.StatsRecorder;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //WriteTransformString();
        
        if (useVectorObs)
        {
            sensor.AddObservation(StepCount / (float)MaxStep);
        }
    }

    IEnumerator GoalScoredSwapGroundMaterial(Material mat, float time)
    {
        m_GroundRenderer.material = mat;
        yield return new WaitForSeconds(time);
        m_GroundRenderer.material = m_GroundMaterial;
    }

    #region Agent movement
    // Seperate functions for either moving the agent through discrete or through continuous actions
    public void MoveAgent_Discrete(ActionSegment<int> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;
        var action = act[0];
        switch (action)
        {
            case 1:
                dirToGo = transform.forward * 1f;
                break;
            case 2:
                dirToGo = transform.forward * -1f;
                break;
            case 3:
                rotateDir = transform.up * 1f;
                break;
            case 4:
                rotateDir = transform.up * -1f;
                break;
        }
        transform.Rotate(rotateDir, Time.deltaTime * 200f);
        m_AgentRb.AddForce(dirToGo * m_HallwaySettings.agentRunSpeed, ForceMode.VelocityChange);
    }

    public void MoveAgent_Continuous(ActionSegment<float> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        rotateDir = transform.up * act[0];
        dirToGo = transform.forward * act[1];

        //WriteActionString(act[1], act[0]);

        transform.Rotate(rotateDir, Time.deltaTime * 200f);
        m_AgentRb.AddForce(dirToGo * m_HallwaySettings.agentRunSpeed, ForceMode.VelocityChange);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)

    {
        AddReward(-1f / MaxStep);

        if (UseContinuousActions)
        {
            MoveAgent_Continuous(actionBuffers.ContinuousActions);
        }
        
        else
        {
            MoveAgent_Discrete(actionBuffers.DiscreteActions);
        }
        
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (UseContinuousActions)
        {
            var continuousActions = actionsOut.ContinuousActions;
            continuousActions[0] = Input.GetAxisRaw("Horizontal");
            continuousActions[1] = Input.GetAxisRaw("Vertical");
        }
        else
        {
            var discreteActionsOut = actionsOut.DiscreteActions;
            if (Input.GetKey(KeyCode.RightArrow))
            {
                discreteActionsOut[0] = 3;
            }
            else if (Input.GetKey(KeyCode.UpArrow))
            {
                discreteActionsOut[0] = 1;
            }
            else if (Input.GetKey(KeyCode.LeftArrow))
            {
                discreteActionsOut[0] = 4;
            }
            else if (Input.GetKey(KeyCode.DownArrow))
            {
                discreteActionsOut[0] = 2;
            }
        }
    }
    #endregion

    void OnCollisionEnter(Collision col)
    {
        if (col.gameObject.CompareTag("symbol_O_Goal") || col.gameObject.CompareTag("symbol_X_Goal"))
        {
            if ((m_Selection == 0 && col.gameObject.CompareTag("symbol_O_Goal")) ||
                (m_Selection == 1 && col.gameObject.CompareTag("symbol_X_Goal")))
            {
                SetReward(1f);
                StartCoroutine(GoalScoredSwapGroundMaterial(m_HallwaySettings.goalScoredMaterial, 0.5f));
                //m_statsRecorder.Add("Goal/Correct", 1, StatAggregationMethod.Sum);
            }
            else
            {
                SetReward(-0.1f);
                StartCoroutine(GoalScoredSwapGroundMaterial(m_HallwaySettings.failMaterial, 0.5f));
                //m_statsRecorder.Add("Goal/Wrong", 1, StatAggregationMethod.Sum);
            }
            EndEpisode();
        }
    }

    public override void OnEpisodeBegin()
    {
        var agentOffset = -15f;
        var blockOffset = 0f;
        m_Selection = Random.Range(0, 2);
        if (m_Selection == 0)
        {
            symbolO.transform.position =
                new Vector3(0f + Random.Range(-3f, 3f), 2f, blockOffset + Random.Range(-5f, 5f))
                + ground.transform.position;
            symbolX.transform.position =
                new Vector3(0f, -1000f, blockOffset + Random.Range(-5f, 5f))
                + ground.transform.position;
        }
        else
        {
            symbolO.transform.position =
                new Vector3(0f, -1000f, blockOffset + Random.Range(-5f, 5f))
                + ground.transform.position;
            symbolX.transform.position =
                new Vector3(0f, 2f, blockOffset + Random.Range(-5f, 5f))
                + ground.transform.position;
        }

        transform.position = new Vector3(0f + Random.Range(-3f, 3f),
            1f, agentOffset + Random.Range(-5f, 5f))
            + ground.transform.position;
        transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        m_AgentRb.velocity *= 0f;

        var goalPos = Random.Range(0, 2);
        if (goalPos == 0)
        {
            symbolOGoal.transform.position = new Vector3(7f, 0.5f, 22.29f) + area.transform.position;
            symbolXGoal.transform.position = new Vector3(-7f, 0.5f, 22.29f) + area.transform.position;
        }
        else
        {
            symbolXGoal.transform.position = new Vector3(7f, 0.5f, 22.29f) + area.transform.position;
            symbolOGoal.transform.position = new Vector3(-7f, 0.5f, 22.29f) + area.transform.position;
        }
        //m_statsRecorder.Add("Goal/Correct", 0, StatAggregationMethod.Sum);
        //m_statsRecorder.Add("Goal/Wrong", 0, StatAggregationMethod.Sum);
    }

    public void WriteActionString(float translation, float rotation)
    {
        string path = Application.persistentDataPath + "/hallway_actions_log.csv";

        StreamWriter writer = new StreamWriter(path, true);
        var LineToWrite = translation + ";" + rotation;
        writer.WriteLine(LineToWrite);
        writer.Close();
    }

    public void WriteTransformString()
    {
        string path = Application.persistentDataPath + "/hallway_transforms_log.csv";

        StreamWriter writer = new StreamWriter(path, true);
        var LineToWrite = transform.localPosition.x + ";" + transform.localPosition.y + ";" + transform.localPosition.z;
        writer.WriteLine(LineToWrite);
        writer.Close();
    }
}
