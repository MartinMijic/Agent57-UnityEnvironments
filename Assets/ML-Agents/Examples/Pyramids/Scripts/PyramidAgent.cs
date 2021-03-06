using System;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.IO;

public class PyramidAgent : Agent
{
    public GameObject area;
    PyramidArea m_MyArea;
    Rigidbody m_AgentRb;
    PyramidSwitch m_SwitchLogic;
    public GameObject areaSwitch;
    public bool useVectorObs;
    [Header("Action Properties")]
    [SerializeField] private bool UseContinuousActions = true;

    public override void Initialize()
    {
        m_AgentRb = GetComponent<Rigidbody>();
        m_MyArea = area.GetComponent<PyramidArea>();
        m_SwitchLogic = areaSwitch.GetComponent<PyramidSwitch>();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //WriteTransformString();

        if (useVectorObs)
        {
            sensor.AddObservation(m_SwitchLogic.GetState());
            sensor.AddObservation(transform.localRotation);
            sensor.AddObservation(transform.InverseTransformDirection(m_AgentRb.velocity));
            //sensor.AddObservation(StepCount / (float)MaxStep);
        }

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
        m_AgentRb.AddForce(dirToGo * 2f, ForceMode.VelocityChange);
    }

    public void MoveAgent_Continuous(ActionSegment<float> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        rotateDir = transform.up * act[0];
        dirToGo = transform.forward * act[1];

        //WriteActionString(act[1], act[0]);

        transform.Rotate(rotateDir, Time.deltaTime * 200f);
        m_AgentRb.AddForce(dirToGo * 2f, ForceMode.VelocityChange);
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

    public override void OnEpisodeBegin()
    {
        var enumerable = Enumerable.Range(0, 9).OrderBy(x => Guid.NewGuid()).Take(9);
        var items = enumerable.ToArray();

        m_MyArea.CleanPyramidArea();

        m_AgentRb.velocity = Vector3.zero;
        m_MyArea.PlaceObject(gameObject, items[0]);
        transform.rotation = Quaternion.Euler(new Vector3(0f, Random.Range(0, 360)));

        m_SwitchLogic.ResetSwitch(items[1], items[2]);
        m_MyArea.CreateStonePyramid(1, items[3]);
        m_MyArea.CreateStonePyramid(1, items[4]);
        m_MyArea.CreateStonePyramid(1, items[5]);
        m_MyArea.CreateStonePyramid(1, items[6]);
        m_MyArea.CreateStonePyramid(1, items[7]);
        m_MyArea.CreateStonePyramid(1, items[8]);
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("goal"))
        {
            SetReward(2f);
            EndEpisode();
        }
    }

    public void WriteTransformString()
    {
        string path = Application.persistentDataPath + "/pyramids_transforms_log.csv";

        StreamWriter writer = new StreamWriter(path, true);
        var LineToWrite = transform.localPosition.x + ";" + transform.localPosition.y + ";" + transform.localPosition.z;
        writer.WriteLine(LineToWrite);
        writer.Close();
    }

    public void WriteActionString(float translation, float rotation)
    {
        string path = Application.persistentDataPath + "/pyramids_actions_log.csv";

        StreamWriter writer = new StreamWriter(path, true);
        var LineToWrite = translation + ";" + rotation;
        writer.WriteLine(LineToWrite);
        writer.Close();
    }

}
