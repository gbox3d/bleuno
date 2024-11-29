#include "Config.hpp"

bool isNumber(String str)
{
    for (int i = 0; i < str.length(); i++)
    {
        if (!isDigit(str[i]))
        {
            return false;
        }
    }
    return true;
}


// parseCmd 함수 구현
void Config::parseCmd(std::vector<String> &tokens, JsonDocument &_res_doc)
{
    int _tokienCount = tokens.size();

    if (_tokienCount > 1)
    {
        String subCmd = tokens[1];
        if (subCmd == "load")
        {
            load();
            _res_doc["result"] = "ok";
            _res_doc["ms"] = "config loaded";
        }
        else if (subCmd == "save")
        {
            save();
            _res_doc["result"] = "ok";
            _res_doc["ms"] = "config saved";
        }
        else if (subCmd == "dump")
        {
            String jsonStr = dump();
            DeserializationError error = deserializeJson(_res_doc["ms"], jsonStr);
            if (error)
            {
                _res_doc["result"] = "fail";
                _res_doc["ms"] = "json parse error";
            }
            else
            {
                _res_doc["result"] = "ok";
            }
        }
        else if (subCmd == "clear")
        {
            clear();
            _res_doc["result"] = "ok";
            _res_doc["ms"] = "config cleared";
        }
        else if (subCmd == "set")
        {
            if (_tokienCount > 2)
            {
                String key = tokens[2];
                String value = tokens[3];

                if(isNumber(value)){
                    set(key.c_str(), value.toInt());
                } else{
                    set(key.c_str(), value);
                }

                // set(key.c_str(), value);

                _res_doc["result"] = "ok";
                _res_doc["ms"] = "config set";
            }
            else
            {
                _res_doc["result"] = "fail";
                _res_doc["ms"] = "need key and value";
            }
        }
        else if (subCmd == "setA")
        { // set json array
            if (_tokienCount > 2)
            {
                String key = tokens[2];
                String value = tokens[3];

                // parse json value
                //  JSON 문자열 파싱을 위한 임시 객체
                JsonDocument tempDoc; // 임시 JSON 문서

                // JSON 문자열 파싱
                DeserializationError error = deserializeJson(tempDoc, value);

                if (error)
                {
                    _res_doc["result"] = "fail";
                    _res_doc["ms"] = "json parse error";
                }
                else
                {
                    // JsonArray array = tempDoc[key].as<JsonArray>();
                    set(key.c_str(), tempDoc);
                    _res_doc["result"] = "ok";
                    _res_doc["ms"] = tempDoc;
                }
            }
            else
            {
                _res_doc["result"] = "fail";
                _res_doc["ms"] = "need key and value";
            }
        }
        else if (subCmd == "get")
        {
            if (_tokienCount > 2)
            {
                String key = tokens[2];

                // check key exist
                if (!hasKey(key.c_str()))
                {
                    _res_doc["result"] = "fail";
                    _res_doc["ms"] = "key not exist";
                }
                else
                {
                    _res_doc["result"] = "ok";
                    _res_doc["value"] = get<String>(key.c_str());
                }
            }
            else
            {
                _res_doc["result"] = "fail";
                _res_doc["ms"] = "need key";
            }
        }
        else
        {
            _res_doc["result"] = "fail";
            _res_doc["ms"] = "unknown sub command";
        }
    }
    else
    {
        _res_doc["result"] = "fail";
        _res_doc["ms"] = "need sub command";
    }
}
