using UnityEngine;
using System.Collections;

/*implements a simple AI behaviour*/
/*should only follow player for now*/

[RequireComponent(typeof(Rigidbody2D))]
public class MouseAI : MonoBehaviour {

    public Rigidbody2D targetBody;                 //target could be player body
    private Rigidbody2D mouseBody;
    private SpriteRenderer mouseRenderer;
    
    private Transform playerTransform;            //needed transform of player itself
    private Transform mouseTransform;

    private bool isPlayerDead = false;
    private bool isMouseDead = false;
    private bool isPlayerVisible = true;            //true for now, due to lazyness -> will be implemented later on... hopefully

    public float moveSpeed = 2.5f;
    public float rotationSpeed = 0.0f;

    private float lifeTime;                        //NOT IMPLEMENTED: lifetime could be for how long the 'uncontrolled' enemy stays visible or etc...

	// Use this for initialization
	void Start () {
        mouseBody = GetComponent<Rigidbody2D>();                        //init all needed variables to steer this enemy
        mouseTransform = mouseBody.GetComponent<Transform>();
        mouseRenderer = GetComponent<SpriteRenderer>();

        playerTransform = targetBody.transform;
	}
	
	// Update is called once per frame
	void Update () {
	    if(targetBody != null)
        {
            //if player exists then follow him
            move(mouseTransform.position, playerTransform.position);
        }
	}


    /*
    * this will move the mouse near the desired position
    *
    */
    void move(Vector2 pos, Vector2 towards)
    {
        Vector2 direction = (towards - pos).normalized;

        if (direction.x < 0)            //flip frames
        {
            mouseRenderer.flipX = false;
        }
        if(direction.x >= 0)
        {
            mouseRenderer.flipX = true;
        }

        mouseBody.MovePosition(mouseBody.position + direction * moveSpeed * Time.deltaTime);           //move towards position on every update using dT       
    }

}
