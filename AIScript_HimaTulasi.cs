using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class AIScript_HimaTulasi : MonoBehaviour {

    public CharacterScript mainScript;

    public float[] bombSpeeds;
    public float[] buttonCooldowns;
    public float playerSpeed;
    public int[] beltDirections;
    public float[] buttonLocations;

    // Use this for initialization
    void Start () {
        mainScript = GetComponent<CharacterScript>();

        if (mainScript == null)
        {
            print("No CharacterScript found on " + gameObject.name);
            this.enabled = false;
        }

        buttonLocations = mainScript.getButtonLocations();

        playerSpeed = mainScript.getPlayerSpeed();
    }

    int targetBeltIndex = 0;

    // finds the closest belt/bomb to the given position
    int GetClosestBeltIndex(float position)
    {
        int closestIndex = -1;
        float minDistance = float.MaxValue;

        // iterate through all buttons to find the closest one to the given position
        for (int i = 0; i < buttonLocations.Length; i++)
        {
            float curDistance = Mathf.Abs(buttonLocations[i] - position);
            if (curDistance < minDistance)
            {
                closestIndex = i;
                minDistance = curDistance;
            }
        }

        return closestIndex;
    }

    // calculates time until the bomb reaches the target button
    float TimeUntilBombReachesButton(int buttonIndex)
    {
        if (buttonIndex < 0 || buttonIndex >= buttonLocations.Length)
            return Mathf.Infinity;

        return mainScript.getBombDistances()[buttonIndex] / mainScript.getBombSpeeds()[buttonIndex];
    }

    // calculates time until the player reaches the target button
    float TimeUntilPlayerReachesButton(int buttonIndex)
    {
        if (buttonIndex < 0 || buttonIndex >= buttonLocations.Length)
            return Mathf.Infinity;

        return Mathf.Abs(mainScript.getCharacterLocation() - buttonLocations[buttonIndex]) / playerSpeed;
    }

    // Update is called once per frame
    void Update () {
        // gets the current game state information
        buttonCooldowns = mainScript.getButtonCooldowns();
        beltDirections = mainScript.getBeltDirections();
        bombSpeeds = mainScript.getBombSpeeds();

        // finds the closest button that is not on cooldown and not moving away
        float minDistance = float.MaxValue;
        int minIndex = -1;

        // iterates through all buttons to find the closest one that can be pressed
        for (int i = 0; i < buttonLocations.Length; i++)
        {
            float curDistance = Mathf.Abs(buttonLocations[i] - mainScript.getCharacterLocation());
            if (buttonCooldowns[i] <= 0 && (beltDirections[i] == -1 || beltDirections[i] == 0))
            {
                if (curDistance < minDistance)
                {
                    minIndex = i;
                    minDistance = curDistance;
                }
            }
        }

        if (minIndex == -1)
        {
            // if no good button is found, do nothing
            return;
        }

        targetBeltIndex = minIndex;

        // calculates time until the bomb reaches a button and time until a player reaches a button
        float bombTime = TimeUntilBombReachesButton(targetBeltIndex);
        float playerTime = TimeUntilPlayerReachesButton(targetBeltIndex);

        // used to make a decision, can the button be pressed on time or should the bomb be avoided
        bool canMakeIt = playerTime + 0.35f < bombTime;
        bool onTarget = targetBeltIndex == GetClosestBeltIndex(mainScript.getOpponentLocation());

        if (canMakeIt || onTarget)
        {
            // move towar a target button to push
            if (buttonLocations[targetBeltIndex] < mainScript.getCharacterLocation())
            {
                mainScript.moveDown();
            }
            else
            {
                mainScript.moveUp();
            }

            mainScript.push();
        }
        else
        {
            // moves away from closest bomb
            int closestBombIndex = GetClosestBeltIndex(mainScript.getCharacterLocation());
            if (closestBombIndex != -1)
            {
                // moves in the opposite direction of the bombs
                if (bombSpeeds[closestBombIndex] > 0)
                {
                    mainScript.moveUp();
                }
                else
                {
                    mainScript.moveDown();
                }
            }
        }
    }
}
