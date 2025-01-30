using System;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;

class Program
{
    static async Task Main(string[] args)
    {
        // Define MQTT broker details (change IP to ROS2 machine's IP)
        string brokerIp = "192.168.2.100"; // Replace with ROS2 machine IP
        int brokerPort = 1883;
        string topic1 = "ros2/mqtt/topic";     // Topic for standard ROS2 messages
        string topic2 = "ros2/mqtt/altitude";  // Topic for altitude values

        // Create an MQTT client
        var factory = new MqttFactory();
        var mqttClient = factory.CreateMqttClient();

        // Configure MQTT client options
        var options = new MqttClientOptionsBuilder()
            .WithTcpServer(brokerIp, brokerPort)
            .WithClientId("CSharpSubscriber")
            .WithCleanSession()
            .Build();

        // Event handler for received messages (Handles both topics)
        mqttClient.UseApplicationMessageReceivedHandler(e =>
        {
            string receivedMessage = Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
            string topic = e.ApplicationMessage.Topic;

            if (topic == topic1)
            {
                Console.WriteLine($"Topic: {topic} - {receivedMessage}");
                System.Diagnostics.Debug.WriteLine($"Received: {receivedMessage} on topic: {topic}");

                if (receivedMessage == "ROS2 Publisher Disconnected")
                {
                    Console.WriteLine("⚠️ WARNING: ROS2 Publisher has disconnected!");
                }
            }
            else if (topic == topic2)
            {
                // Try parsing the altitude value as a float
                if (float.TryParse(receivedMessage, System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out float altitudeValue))
                {
                    Console.WriteLine($"Topic: {topic} - Altitude: {altitudeValue:F2}m");
                }
                else
                {
                    Console.WriteLine($"[ERROR] Failed to parse altitude value: {receivedMessage}");
                }
            }
        });


        // Event handler for connection
        mqttClient.UseConnectedHandler(async e =>
        {
            Console.WriteLine("Connected to MQTT broker.");

            // Subscribe to both topics
            await mqttClient.SubscribeAsync(topic1);
            await mqttClient.SubscribeAsync(topic2);

            Console.WriteLine($"Subscribed to topics: {topic1}, {topic2}");
        });

        // Event handler for disconnection (with reconnection logic)
        mqttClient.UseDisconnectedHandler(async e =>
        {
            Console.WriteLine("Disconnected from MQTT broker. Reconnecting...");
            await Task.Delay(5000); // Wait before reconnecting
            try
            {
                await mqttClient.ConnectAsync(options);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Reconnection failed: {ex.Message}");
            }
        });

        // Connect to MQTT broker
        Console.WriteLine("Connecting to MQTT broker...");
        await mqttClient.ConnectAsync(options);

        // Keep the application running
        Console.WriteLine("Press any key to exit...");
        Console.ReadLine();

        // Disconnect before exiting
        await mqttClient.DisconnectAsync();
    }
}




