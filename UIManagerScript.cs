using UnityEngine;
using System.Collections;
using UnityEngine.SceneManagement;

public class UIManagerScript : MonoBehaviour {

    public Animator startButton;
    public Animator settingsButton;
    public Animator dialog;

    public void StartGame()
    {
        SceneManager.LoadScene("SuperAlienChickenHorse");
    }

    public void OpenSettings()
    {
        startButton.SetBool("isHidden", true);
        settingsButton.SetBool("isHidden", true);
        dialog.SetBool("isHidden", false);
    }

    public void CloseSettings()
    {
        startButton.SetBool("isHidden", false);
        settingsButton.SetBool("isHidden", false);
        dialog.SetBool("isHidden", true);
    }


    //Quit is ignored in the editor or the web player
    public void OnQuitGame()
    {
        Application.Quit();
    }

}
