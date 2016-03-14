using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/*moves object along a series of waypoints, useful for moving platforms or hazards
this class adds a kinematic rigidbody so the moving object will push other rigidbodies whilst moving*/

public class PlatformMovement : MonoBehaviour
{
    public float moveSpeed;  
    public float delay;                  //stoptime for each waypoint
    public type movementType;            //stop at final waypoint, loop through waypoints or move back & forth along waypoints
    public enum type { PlayOnce, Loop, PingPong }

    private int currentWp;
    private float arrivalTime;

    private bool forward = true, arrived = false;           //arrival at waypoints
    public bool drawGizmos = true;                          //should gizmos be drawn in scene

    private List<Transform> waypoints = new List<Transform>();

    private Rigidbody2D platformBody;


    void Start()
    {
        platformBody = GetComponentInParent<Rigidbody2D>();         //use rigidBody2D from parent
    }

    //setup
    void Awake()
    {
        //get child waypoints, then detach them
        foreach (Transform child in transform)
            if (child.tag == "Waypoint")
            {
                waypoints.Add(child);
            }
        if (waypoints.Count == 0)
        {
            Debug.LogError("No waypoints found -> add child gameObjects with the tag 'Waypoint'", transform);
        }
        transform.DetachChildren();
    }



    //if we've arrived at waypoint, get the next one
    void Update()
    {
        if (waypoints.Count > 0)
        {
            if (!arrived)
            {
                if (Vector3.Distance(transform.position, waypoints[currentWp].position) < 0.3f)
                {
                    arrivalTime = Time.time;
                    arrived = true;
                }
            }
            else
            {
                if (Time.time > arrivalTime + delay)
                {
                    GetNextWP();
                    arrived = false;
                }
            }
        }
    }



    //move object toward waypoint
    void FixedUpdate()
    {
        if (!arrived && waypoints.Count > 0)
        {
            move(transform.position, waypoints[currentWp].position);
        }
    }


    /*
    * the actual method how will steer the platform according to their inherited transformation vectors
    *
    */
    void move(Vector2 pos, Vector2 towards)
    {
        Vector2 direction = (towards - pos).normalized;
        platformBody.MovePosition(platformBody.position + direction * moveSpeed * Time.deltaTime);           //move towards position on every update using dT       
    }



    //get the next waypoint
    private void GetNextWP()
    {
        if (movementType == type.PlayOnce)
        {
            currentWp++;
            if (currentWp == waypoints.Count)
            {
                enabled = false;
            }
        }

        if (movementType == type.Loop)
        {
            currentWp = (currentWp == waypoints.Count - 1) ? 0 : currentWp += 1;
        }

        if (movementType == type.PingPong)
        {
            if (currentWp == waypoints.Count - 1)
            {
                forward = false;
            }
            else if (currentWp == 0)
            {
                forward = true;
            }
            currentWp = (forward) ? currentWp += 1 : currentWp -= 1;
        }
    }



    //draw gizmo spheres for waypoints
    void OnDrawGizmos()
    {
        if (drawGizmos)
        {
            Gizmos.color = Color.gray;
            foreach (Transform child in transform)
            {
                if (child.tag == "Waypoint")
                {
                    //draw a little sphere on waypoints in scene view
                    Gizmos.DrawSphere(child.position, .15f);
                    //additionally draw a line from one wp to another
                 //   for (int i = 0; i != waypoints.Count -1; i++) {
                 //       Gizmos.DrawLine(new Vector3(waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z),
                 //           new Vector3(waypoints[i + 1].position.x, waypoints[i + 1].position.y, waypoints[i + 1].position.z));
                //    }
                }           
            }
        }
    }


}

