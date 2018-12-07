#ifndef EMOTIV_READER_DEF_H_
#define EMOTIV_READER_DEF_H_

int emotiv(ros::Publisher temp_pub);
int validateEmotivJson(Json::Value jsonData);
int validateEmotivJsonActionStatus(Json::Value jsonData);
int validateEmotivJsonPlayers(Json::Value jsonData);
int validateEmotivJsonPlayersSize(Json::Value jsonData);
int validateEmotivJsonPlayerObj(Json::Value jsonData);
int validateEmotivJsonCurrentAction(Json::Value jsonData);
std::string parseCurrentAction(int current_action);

namespace
{
    std::size_t callback(
            const char* in,
            std::size_t size,
            std::size_t num,
            std::string* out)
    {
        const std::size_t totalBytes(size * num);
        out->append(in, totalBytes);
        return totalBytes;
    }
}

#endif
