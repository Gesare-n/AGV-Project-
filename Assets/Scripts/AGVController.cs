using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class AGVController : MonoBehaviour
{
    public string piIP = "192.168.1.100";
    public int piPort = 65432;

    TcpClient client;
    NetworkStream stream;

    void Start()
    {
        ConnectToPi();
    }

    void ConnectToPi()
    {
        try
        {
            client = new TcpClient(piIP, piPort);
            stream = client.GetStream();
            Debug.Log("Connected to Raspberry Pi");
        }
        catch
        {
            Debug.LogError("Failed to connect to Raspberry Pi");
        }
    }

    public void SendMotorCommand(float left, float right)
    {
        if (stream == null) return;

        string cmd = left + " " + right + "\n";
        byte[] data = Encoding.ASCII.GetBytes(cmd);
        try { stream.Write(data, 0, data.Length); }
        catch { Debug.LogWarning("Failed to send motor command"); }
    }

    public void SendArmCommand(float joint1, float joint2, float joint3)
    {
        if (stream == null) return;
        string cmd = $"arm {joint1} {joint2} {joint3}\n";
        byte[] data = Encoding.ASCII.GetBytes(cmd);
        try { stream.Write(data, 0, data.Length); }
        catch { Debug.LogWarning("Failed to send arm command"); }
    }

    public void SendCameraCommand(float pan)
    {
        if (stream == null) return;
        string cmd = $"camera {pan}\n";
        byte[] data = Encoding.ASCII.GetBytes(cmd);
        try { stream.Write(data, 0, data.Length); }
        catch { Debug.LogWarning("Failed to send camera command"); }
    }
}
