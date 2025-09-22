using System;
using System.Collections.Generic;
using UnityEngine;

namespace GlitchSim.Runtime.Communication
{
    /// <summary>
    /// Provides communication to and from a number of robots.
    /// </summary>
    public class RobotCommunication : ICommunication
    {
        public event ICommunication.MessageReceivedHandler OnMessageReceived;
        
        private readonly RobotCommunicationSettings _settings;
        private readonly NetworkTablesWrapper _networkTables = new();
        private readonly List<NetworkTablesWrapper.Subscriber> _subscribers = new();

        public RobotCommunication(RobotCommunicationSettings settings)
        {
            _settings = settings;
            
            _networkTables.Connect("UnityClient", _settings.HostName, _settings.Port);
        }

        public void Subscribe(string topicName, ICommunication.DataType type)
        {
            NetworkTablesWrapper.TopicType topicType;
            
            switch (type)
            {
                case ICommunication.DataType.Boolean:
                    topicType = NetworkTablesWrapper.TopicType.Boolean;
                    break;
                case ICommunication.DataType.Float:
                    topicType = NetworkTablesWrapper.TopicType.Float;
                    break;
                case ICommunication.DataType.Double:
                    topicType = NetworkTablesWrapper.TopicType.Double;
                    break;
                case ICommunication.DataType.Unsupported:
                    return;
                default:
                    throw new ArgumentOutOfRangeException(nameof(type), type, null);
            }
            
            // Avoid duplicates
            foreach (var subscriber in _subscribers)
            {
                if (subscriber.TopicName == topicName && subscriber.Type == topicType)
                {
                    return;
                }
            }
            
            _subscribers.Add(_networkTables.Subscribe(topicName, topicType));
        }

        public void UpdateSubscribers()
        {
            if (OnMessageReceived == null)
            {
                return;
            }
            
            foreach (var subscriber in _subscribers)
            {
                OnMessageReceived.Invoke(subscriber.GetMessage());
            }
        }
        
        public void SendMessage(IMessage message)
        {
            // TODO
        }
    }
}
