using UnityEngine;
using System.Collections;

/*very simple raycast which represents the enemy line of sight */

[RequireComponent(typeof(LayerMask))]
[RequireComponent(typeof(Rigidbody2D))]
public class EnemySight : MonoBehaviour {

    public LayerMask LayerReactor;

    public Transform sightStart;
    public Transform sightEnd;

    public bool playerInSight = false;

    public bool facingLeft = true;

    private Rigidbody2D enemyBody;

    public GameObject target;

    private Transform playerTransform;
    private Rigidbody2D playerBody;

    public float moveSpeed = 2.5f;
    public float rotationSpeed = 0.0f;

    private float lifeTime;                        //NOT IMPLEMENTED: lifetime could be for how long the 'uncontrolled' enemy stays visible or etc...


    void Start()
    {
        playerTransform = target.GetComponent<Transform>();
        playerBody = target.GetComponent<Rigidbody2D>();

        enemyBody = GetComponent<Rigidbody2D>();
        InvokeRepeating("Patrol", 0.0f, Random.Range(2.0f, 6.0f));
    }

    void Update()
    {
        RaycastSight();
        Behaviours();
    }

    void RaycastSight()
    {
        Debug.DrawLine(sightStart.position, sightEnd.position, Color.red);
        playerInSight = Physics2D.Linecast(sightStart.position, sightEnd.position, 1 << LayerMask.NameToLayer("Player"));
    }

    void Behaviours()
    {
        if (playerInSight)
        {
            //if player in sight, then follow him immediately
            followTarget();
        }
    }


    void followTarget()
    {
        move(enemyBody.transform.position, playerTransform.position);
    }
  



    void Patrol()
    {
        facingLeft = !facingLeft;

        if (facingLeft)
        {
            enemyBody.transform.eulerAngles = new Vector2(0f, 0f);
        }
        else
        {
            enemyBody.transform.eulerAngles = new Vector2(0, 180f);
        }
    }



    /*
    * this will move the mouse near the desired position
    *
    */
    void move(Vector2 pos, Vector2 towards)
    {
        Vector2 direction = (towards - pos).normalized;

        enemyBody.MovePosition(enemyBody.position + direction * moveSpeed * Time.deltaTime);           //move towards position on every update using dT       
    }
}
