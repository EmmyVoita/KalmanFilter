using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GraphController : MonoBehaviour
{
    [SerializeField]private Sprite circleSprite;
    private RectTransform graphContainer;
    public Main main;



    private void Awake()
    {
        graphContainer = transform.Find("graphContainer").GetComponent<RectTransform>();
        //CreateCircle(new Vector2(200,200));
    }


    private GameObject CreateCircle(Vector2 anchoredPosition)
    {
        GameObject gameObject = new GameObject("circle", typeof(Image));
        gameObject.transform.SetParent(graphContainer, false);
        gameObject.GetComponent<Image>().sprite = circleSprite;
        RectTransform rectTransform = gameObject.GetComponent<RectTransform>();
        rectTransform.anchoredPosition = anchoredPosition;
        rectTransform.sizeDelta = new Vector2(6, 6);
        rectTransform.anchorMin = new Vector2(0, 0);
        rectTransform.anchorMax = new Vector2(0, 0);
        return gameObject;
    }

    public void ShowGraph(List<float> valueList, List<float> valueList2, List<float> valueList3)
    {
        foreach (Transform child in graphContainer.transform)
        {
            if(child.transform.name == "circle" || child.transform.name == "dotConnection")
            GameObject.Destroy(child.gameObject);
        }

        float graphHeight = graphContainer.sizeDelta.y;
        float yMaximum = 20f;


        float xSize = 30f;
        GameObject lastCircleGameObject = null;
        GameObject lastCircleGameObject2 = null;
        GameObject lastCircleGameObject3 = null;
        for (int i = 0; i < valueList.Count; i++)
        {
            float xPosition = i * xSize;
            float yPosition = (valueList[i]/ yMaximum) * graphHeight;
            float yPosition2 = (valueList2[i] / yMaximum) * graphHeight;
            float yPosition3 = (valueList3[i] / yMaximum) * graphHeight;

            GameObject circleGameObject = CreateCircle(new Vector2(xPosition, yPosition));

            if (lastCircleGameObject != null)
            {
                CreateDotConnection(lastCircleGameObject.GetComponent<RectTransform>().anchoredPosition, circleGameObject.GetComponent<RectTransform>().anchoredPosition, new Color(1, 0, 0, 0.5f));
            }
            lastCircleGameObject = circleGameObject;

            GameObject circleGameObject2 = CreateCircle(new Vector2(xPosition, yPosition2));

            if (lastCircleGameObject2 != null)
            {
                CreateDotConnection(lastCircleGameObject2.GetComponent<RectTransform>().anchoredPosition, circleGameObject2.GetComponent<RectTransform>().anchoredPosition, new Color(0, 1, 0, 0.5f));
            }
            lastCircleGameObject2 = circleGameObject2;

            GameObject circleGameObject3 = CreateCircle(new Vector2(xPosition, yPosition3));

            if (lastCircleGameObject3 != null)
            {
                CreateDotConnection(lastCircleGameObject3.GetComponent<RectTransform>().anchoredPosition, circleGameObject3.GetComponent<RectTransform>().anchoredPosition, new Color(0, 0, 1, 0.5f));
            }
            lastCircleGameObject3 = circleGameObject3;
        }
    }

    public float CalculateAngle(Vector2 direction, float distance)
    {
        float angle = Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg;
        return angle < 0 ? angle + 360 : angle;
    }


    private void CreateDotConnection(Vector2 dotPositionA, Vector2 dotPositionB, Color color)
    {
        GameObject gameObject = new GameObject("dotConnection", typeof(Image));
        gameObject.transform.SetParent(graphContainer, false);
        gameObject.GetComponent<Image>().sprite = circleSprite;
        gameObject.GetComponent<Image>().color = color;

        Vector2 dir = (dotPositionB - dotPositionA).normalized;
        float distance = Vector2.Distance(dotPositionA, dotPositionB);

        RectTransform rectTransform = gameObject.GetComponent<RectTransform>();
        rectTransform.sizeDelta = new Vector2(distance, 3f);
        rectTransform.anchorMin = new Vector2(0, 0);
        rectTransform.anchorMax = new Vector2(0, 0);
        rectTransform.anchoredPosition = dotPositionA + dir * distance * 0.5f;
        rectTransform.localEulerAngles = new Vector3(0, 0, CalculateAngle(dir, distance));

    }
}
