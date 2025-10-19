using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class CameraFeed : MonoBehaviour
{
    public string streamUrl = "http://<raspberrypi_ip>:5000/video_feed";
    public RawImage rawImage;

    void Start()
    {
        StartCoroutine(StreamCamera());
    }

    IEnumerator StreamCamera()
    {
        while (true)
        {
            using (var www = UnityEngine.Networking.UnityWebRequestTexture.GetTexture(streamUrl))
            {
                yield return www.SendWebRequest();
                if (www.result != UnityEngine.Networking.UnityWebRequest.Result.Success)
                    Debug.Log(www.error);
                else
                    rawImage.texture = UnityEngine.Networking.DownloadHandlerTexture.GetContent(www);
            }
        }
    }
}
