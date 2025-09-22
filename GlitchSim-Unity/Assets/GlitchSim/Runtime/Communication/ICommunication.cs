namespace GlitchSim.Runtime.Communication
{
    public interface IMessage
    {
        public string ToString();
    }
    
    /// <summary>
    /// Interface for a system that communicates with something in the outside world.
    /// </summary>
    public interface ICommunication
    {
        public enum DataType
        {
            Boolean,
            Float,
            Double,
            Unsupported
        }
        
        /// <summary>
        /// Delegate that represents a method that handles a received message.
        /// </summary>
        /// <param name="message">The received message.</param>
        public delegate void MessageReceivedHandler(IMessage message);

        /// <summary>
        /// Event invoked when a new message is received.
        /// </summary>
        public event MessageReceivedHandler OnMessageReceived;

        /// <summary>
        /// Subscribes to a specific topic with a specified data type.
        /// </summary>
        /// <param name="topicName">The name of the topic to subscribe to.</param>
        /// <param name="type">The data type of the topic being subscribed to.</param>
        public void Subscribe(string topicName, DataType type);

        /// <summary>
        /// Send a message.
        /// </summary>
        /// <param name="message">The message to send.</param>
        public void SendMessage(IMessage message);
    }
}
