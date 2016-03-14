using UnityEngine;
using System.Collections;

public class HoldCharacter : MonoBehaviour {

 //   public GameObject player;


  //  private Rigidbody2D rigidPlatform;
   // private Rigidbody2D rigidPlayer;

    private bool isOnPlatform;

    void Start()
    {
        //     rigidPlayer = player.GetComponent<Rigidbody2D>();
        isOnPlatform = false;
    }


 /*   void FixedUpdate()
    {
        if(isOnPlatform)
        {
           // rigidPlayer.MovePosition(platformPos.position);

          //  playerPos.position.Set(platformPos.position.x + playerPos.position.x, platformPos.position.y + playerPos.position.y, 0f);
        } 
    }*/


    void OnTriggerEnter2D() {
        isOnPlatform = true;
    }

    void onTriggerExit2D() {
        isOnPlatform = false;
    }

}
