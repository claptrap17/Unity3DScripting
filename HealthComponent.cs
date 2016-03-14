using UnityEngine;
using System.Collections;

[RequireComponent(typeof(BoxCollider2D))]
public class HealthComponent : MonoBehaviour {

    public int HealthPoints = 100;

    public BoxCollider2D colPlayer;

    private BoxCollider2D colEnemy;

    private bool touched = false;
    private bool isAlive = true;

	// Use this for initialization
	void Start () {
        colEnemy = GetComponent<BoxCollider2D>();
	}
	


	// Update is called once per frame
	void Update () {
        if (colEnemy.IsTouching(colPlayer))
        {
            touched = true;
        }
        else
        {
            touched = false;
        }
        setHealthPoints(touched);
	}


    void setHealthPoints(bool hit)
    {
        if (hit && isAlive)
        {
            HealthPoints -= 10;
        }
        isObjectAlive();
    }



    void isObjectAlive()
    {
        if(HealthPoints >= 1)
        {
            isAlive = true;
        }
        else
        {
            isAlive = false;
        }
    }



    
}
