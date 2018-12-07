#include <sstream>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <jsoncpp/json/json.h>
#include <curl/curl.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "jogo_quente_frio/emotiv_reader.h"


int main(int argc, char **argv)
{       
    ros::init(argc, argv, "emotiv_reader");

    ros::NodeHandle n;

    //ros::Publisher temp_pub = n.advertise<std_msgs::String>("temperatura", 1000);
    ros::Publisher temp_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        emotiv(temp_pub);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

int emotiv(ros::Publisher temp_pub) {

    const std::string url("http://www.mocky.io/v2/5c0ac0683500005700a86296");

    CURL* curl = curl_easy_init();

    // Set remote URL.
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    // Don't bother trying IPv6, which would increase DNS resolution time.
    curl_easy_setopt(curl, CURLOPT_IPRESOLVE, CURL_IPRESOLVE_V4);

    // Don't wait forever, time out after 10 seconds.
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10);

    // Follow HTTP redirects if necessary.
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    // Response information.
    int httpCode(0);
    std::string* httpData(new std::string());

    // Hook up data handling function.
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback);

    // Hook up data container (will be passed as the last parameter to the
    // callback handling function).  Can be any pointer type, since it will
    // internally be passed as a void pointer.
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, httpData);

    // Run our HTTP GET command, capture the HTTP response code, and clean up.
    curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
    curl_easy_cleanup(curl);

    if (httpCode == 200)
    {
        // Response looks good - done using Curl now.  Try to parse the results
        Json::Value jsonData;
        Json::Reader jsonReader;

        // Parse json retrived from emotiv server
        if (jsonReader.parse(*httpData, jsonData))
        {
            // Check errors at retrived json
            if(validateEmotivJson(jsonData))
            {
                // If errors return
                return 1;
            }

            // Read trained command from parsed json
            const int current_action(jsonData["action_status"]["players"][0]["current_action"].asInt());

            // Ros message to be published to temperatura topic
            std_msgs::String rosMessage;

            // Convert current_action code to string (stop, hot or cold) and prepare ros message
            rosMessage.data = parseCurrentAction(current_action);

            // Publish command
            temp_pub.publish(rosMessage);


        }
        else
        {
            std::cout << "Could not parse HTTP data as JSON" << std::endl;
            std::cout << "HTTP data was:\n" << *httpData << std::endl;
            return 1;
        }
    }
    else
    {
        std::cout << "Couldn't GET from " << url << " - exiting" << std::endl;
        return 1;
    }

    return 0;
}

int validateEmotivJson(Json::Value jsonData)
{
    if(validateEmotivJsonActionStatus(jsonData)) {
        return 1;
    }

    if(validateEmotivJsonPlayers(jsonData)) {
        return 1;
    }

    if(validateEmotivJsonPlayersSize(jsonData)) {
        return 1;
    }

    if(validateEmotivJsonPlayerObj(jsonData)) {
        return 1;
    }

    if(validateEmotivJsonCurrentAction(jsonData)) {
        return 1;
    }

    //Validation OK
    return 0;
}

int validateEmotivJsonActionStatus(Json::Value jsonData) {

    if(jsonData["action_status"].isObject())
    {
        return 0;
    }
    else
    {
        std::cout << "Error validating Emotiv JSON: Field action_status is not an Object"  << std::endl;
        std::cout << jsonData.toStyledString() << std::endl;
        return 1;
    }
}

int validateEmotivJsonPlayers(Json::Value jsonData) {

    if(jsonData["action_status"]["players"].isArray())
    {
        return 0;
    }
    else
    {
        std::cout << "Error validating Emotiv JSON: Field action_status->players is not an Array"  << std::endl;
        std::cout << jsonData.toStyledString() << std::endl;
        return 1;
    }
}

int validateEmotivJsonPlayersSize(Json::Value jsonData) {

    if(jsonData["action_status"]["players"].size() > 0)
    {
        return 0;
    }
    else
    {
        std::cout << "Error validating Emotiv JSON: Field action_status->players is empty"  << std::endl;
        std::cout << jsonData.toStyledString() << std::endl;
        return 1;
    }
}

int validateEmotivJsonPlayerObj(Json::Value jsonData) {

    if(jsonData["action_status"]["players"][0].isObject())
    {
        return 0;
    }
    else
    {
        std::cout << "Error validating Emotiv JSON: Field action_status->players[0] is not an Object"  << std::endl;
        std::cout << jsonData.toStyledString() << std::endl;
        return 1;
    }
}

int validateEmotivJsonCurrentAction(Json::Value jsonData) {

    if(jsonData["action_status"]["players"][0]["current_action"].isInt())
    {
        return 0;
    }
    else
    {
        std::cout << "Error validating Emotiv JSON: Field action_status->players[0]->current_action is not an Integer"  << std::endl;
        std::cout << jsonData.toStyledString() << std::endl;
        return 1;
    }
}

// Convert current_action (S)top, (Q)uente or (F)rio
std::string parseCurrentAction(int current_action)
{
    switch (current_action)
    {
        case 0:
            //stop
            return "S";
        case 1:
            //quente
            return "Q";
        case 2:
            //frio
            return "F";
        default:
            return "";
    }
}
