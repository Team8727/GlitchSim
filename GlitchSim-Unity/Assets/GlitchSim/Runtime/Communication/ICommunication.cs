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
        /// Send a message.
        /// </summary>
        /// <param name="message">The message to send.</param>
        public void SendMessage(IMessage message);
    }
}
