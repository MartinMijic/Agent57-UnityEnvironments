using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

//25.08.2021: Actions auf 4 statt 5 angepasst

public class SorterAgent : Agent
{
    [Range(1, 20)]
    public int DefaultMaxNumTiles;
    private const int k_HighestTileValue = 20;

    int m_NumberOfTilesToSpawn;
    int m_MaxNumberOfTiles;
    Rigidbody m_AgentRb;

    // The BufferSensorComponent is the Sensor that allows the Agent to observe
    // a variable number of items (here, numbered tiles)
    //BufferSensorComponent m_BufferSensor;

    public List<NumberTile> NumberTilesList = new List<NumberTile>();

    private List<NumberTile> CurrentlyVisibleTilesList = new List<NumberTile>();
    private List<Transform> AlreadyTouchedList = new List<Transform>();

    private List<int> m_UsedPositionsList = new List<int>();
    private Vector3 m_StartingPos;

    GameObject m_Area;
    EnvironmentParameters m_ResetParams;

    private int m_NextExpectedTileIndex;

    [Header("Action Properties")]
    [SerializeField] private bool UseContinuousActions = true;

    public override void Initialize()
    {
        m_Area = transform.parent.gameObject;
        m_MaxNumberOfTiles = k_HighestTileValue;
        m_ResetParams = Academy.Instance.EnvironmentParameters;
        //m_BufferSensor = GetComponent<BufferSensorComponent>();
        m_AgentRb = GetComponent<Rigidbody>();
        m_StartingPos = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        m_MaxNumberOfTiles = (int)m_ResetParams.GetWithDefault("num_tiles", DefaultMaxNumTiles);

        //m_NumberOfTilesToSpawn = Random.Range(1, m_MaxNumberOfTiles + 1);
        m_NumberOfTilesToSpawn = 3;
        SelectTilesToShow();
        SetTilePositions();

        transform.position = m_StartingPos;
        m_AgentRb.velocity = Vector3.zero;
        m_AgentRb.angularVelocity = Vector3.zero;
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation((transform.position.x - m_Area.transform.position.x) / 20f);
        sensor.AddObservation((transform.position.z - m_Area.transform.position.z) / 20f);

        sensor.AddObservation(transform.forward.x);
        sensor.AddObservation(transform.forward.z);

        foreach (var item in CurrentlyVisibleTilesList)
        {
            // Each observation / tile in the BufferSensor will have 22 values
            // The first 20 are one hot encoding of the value of the tile
            // The 21st and 22nd are the position of the tile relative to the agent
            // The 23rd is a boolean : 1 if the tile was visited already and 0 otherwise
            float[] listObservation = new float[k_HighestTileValue + 3];
            listObservation[item.NumberValue] = 1.0f;
            var tileTransform = item.transform.GetChild(1);
            listObservation[k_HighestTileValue] = (tileTransform.position.x - transform.position.x) / 20f;
            listObservation[k_HighestTileValue + 1] = (tileTransform.position.z - transform.position.z) / 20f;
            listObservation[k_HighestTileValue + 2] = item.IsVisited ? 1.0f : 0.0f;
            sensor.AddObservation(listObservation);
            // Here, the observation for the tile is added to the BufferSensor
            //m_BufferSensor.AppendObservation(listObservation);

        }

        sensor.AddObservation(StepCount / (float)MaxStep);

    }

    private void OnCollisionEnter(Collision col)
    {
        if (!col.gameObject.CompareTag("tile"))
        {
            return;
        }
        if (AlreadyTouchedList.Contains(col.transform))
        {
            return;
        }
        if (col.transform.parent != CurrentlyVisibleTilesList[m_NextExpectedTileIndex].transform)
        {
            // The Agent Failed
            AddReward(-1);
            EndEpisode();
        }
        else
        {
            // The Agent Succeeded
            AddReward(1);
            var tile = col.gameObject.GetComponentInParent<NumberTile>();
            tile.VisitTile();
            m_NextExpectedTileIndex++;

            AlreadyTouchedList.Add(col.transform);

            //We got all of them. Can reset now.
            if (m_NextExpectedTileIndex == m_NumberOfTilesToSpawn)
            {
                EndEpisode();
            }
        }
    }

    #region Tile-specific
    void SetTilePositions()
    {

        m_UsedPositionsList.Clear();

        //Disable all. We will enable the ones selected
        foreach (var item in NumberTilesList)
        {
            item.ResetTile();
            item.gameObject.SetActive(false);
        }


        foreach (var item in CurrentlyVisibleTilesList)
        {
            //Select a rnd spawnAngle
            bool posChosen = false;
            int rndPosIndx = 0;
            while (!posChosen)
            {
                rndPosIndx = Random.Range(0, k_HighestTileValue);
                if (!m_UsedPositionsList.Contains(rndPosIndx))
                {
                    m_UsedPositionsList.Add(rndPosIndx);
                    posChosen = true;
                }
            }
            item.transform.localRotation = Quaternion.Euler(0, rndPosIndx * (360f / k_HighestTileValue), 0);
            item.gameObject.SetActive(true);
        }
    }

    void SelectTilesToShow()
    {

        CurrentlyVisibleTilesList.Clear();
        AlreadyTouchedList.Clear();

        int numLeft = m_NumberOfTilesToSpawn;
        while (numLeft > 0)
        {
            int rndInt = Random.Range(0, k_HighestTileValue);
            var tmp = NumberTilesList[rndInt];
            if (!CurrentlyVisibleTilesList.Contains(tmp))
            {
                CurrentlyVisibleTilesList.Add(tmp);
                numLeft--;
            }
        }

        //Sort Ascending
        CurrentlyVisibleTilesList.Sort((x, y) => x.NumberValue.CompareTo(y.NumberValue));
        m_NextExpectedTileIndex = 0;
    }
    #endregion

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
}
