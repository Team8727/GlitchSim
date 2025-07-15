using System;
using NativeWebSocket;
using UnityEngine;

namespace GlitchSim.Runtime.Communication
{
    public class RobotCommunicationMessage : IMessage
    {
        private readonly string _message;

        public RobotCommunicationMessage(string message)
        {
            _message = message;
        }

        public RobotCommunicationMessage(byte[] data)
        {
            _message = System.Text.Encoding.UTF8.GetString(data);
        }
        
        public override string ToString()
        {
            return _message;
        }
        
        public byte[] ToBytes()
        {
            return System.Text.Encoding.UTF8.GetBytes(_message);
        }
    }
    
    /// <summary>
    /// Provides communication to and from a number of robots.
    /// </summary>
    public class RobotCommunication : ICommunication, IDisposable
    {
        public event ICommunication.MessageReceivedHandler OnMessageReceived;
        
        private readonly RobotCommunicationSettings _settings;
        private readonly WebSocket _webSocket;
        
        public RobotCommunication(RobotCommunicationSettings settings)
        {
            Debug.LogWarning(NetworkTablesWrapper.test());
            _settings = settings;

            // Start the websocket
            {
                _webSocket = new WebSocket(_settings.HostName);
                _webSocket.OnMessage += OnWebsocketMessage;
                _webSocket.Connect();
            }
        }

        public void Dispose()
        {
            _webSocket.Close();
        }

        public void SendMessage(IMessage message)
        {
            if (message is RobotCommunicationMessage robotCommunicationMessage)
            {
                _webSocket.Send(robotCommunicationMessage.ToBytes());
            }
        }

        /// <summary>
        /// Pump the incoming messages
        /// </summary>
        public void DispatchMessageQueue()
        {
            _webSocket.DispatchMessageQueue();
        }
        
        private void OnWebsocketMessage(byte[] data)
        {
            OnMessageReceived?.Invoke(new RobotCommunicationMessage(data));
        }
    }
}
