/*
 * Config.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: lieven
 */

#include "Config.h"
#include <EventBus.h>

#include <Sys.h>
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xDEADBEEF
#define KEY_SIZE 40
#define VALUE_SIZE 60

Config::Config() : _strBuffer(1024),_jsonBuffer(1024),_nameSpace(30),_loaded(false)
{

}

Config::~Config()
{

}

void Config::initMagic()
{
   
}

bool Config::checkMagic()
{   
    return false;
}

void Config::clear()
{
    
    load();
}

void Config::load()
{
    INFO(" config load");
    _loaded=true;
    _jsonBuffer.parse("{}");
    if ( _loaded ) return;
}

const char* Config::clone(const char* s) {
    char* p=(char*)_jsonBuffer.alloc(strlen(s)+1);
    strcpy(p,s);
    return p;
}


void Config::save()
{

    char json[1024];

    /*size_t actualLen = */_root->printTo(json);
    
}

//======================================NAMESPACE 
JsonObject& Config::nameSpace()
{

    if ( _root->containsKey(_nameSpace.c_str())) {
        JsonObject& jso =  (* _root)[_nameSpace.c_str()];
        return jso;
    } else {
        JsonObject& jso =  _root->createNestedObject(clone(_nameSpace.c_str()));
        return jso;
    }
}

void Config:: setNameSpace(const char* ns)
{
    _nameSpace=ns;
}

const char* Config:: getNameSpace()
{
    return   _nameSpace.c_str();
}
//==================================================
bool Config::hasKey(const char* key)
{
    JsonObject& ns=nameSpace();
    if ( ns.containsKey(key)) {
        return true;
    }
    return false;
}

void Config::remove(const char* key)
{
    load();
    JsonObject& ns=nameSpace();
    ns.remove(key);
    INFO(" Config => SAVE  remove %s ",key);

}



void Config::set(const char* key,const char* value)
{
    JsonObject& ns=nameSpace();
    ns[clone(key)]=clone(value);
}

void Config::set(const char* key, Str& value)
{
    load();
    set(key,value.c_str());
}

void Config::set(const char* key, uint32_t value)
{
    load();
    JsonObject& ns=nameSpace();
    ns[clone(key)]=value;
}


void Config::set(const char* key, int32_t value)
{
    load();
    JsonObject& ns=nameSpace();
    ns[clone(key)]=value;
}

void Config::set(const char* key, double value)
{
    load();
    JsonObject& ns=nameSpace();
    INFO(" SET %s=%f ",key,value);
    ns[clone(key)]=value;
}


void Config::get(const char* key, Str& value,
                 const char* defaultValue)
{

    load();
    JsonObject& ns=nameSpace();
    if ( ns.containsKey(key)) {
        value= ns[key];
    } else {
        set(key,defaultValue);
        value=defaultValue;
    }
    INFO(" %s.%s = '%s' ",_nameSpace.c_str(),key,value.c_str());
}


void Config::get(const char* key, int32_t& value,
                 int32_t defaultValue)
{
    load();
    JsonObject& ns=nameSpace();
    if ( ns.containsKey(key)) {
        value= ns[key];
    } else {
        set(key,defaultValue);
        value=defaultValue;
    }
    INFO(" %s.%s = %d ",_nameSpace.c_str(),key,value);
}

void Config::get(const char* key, uint32_t& value,
                 uint32_t defaultValue)
{
    load();
    JsonObject& ns=nameSpace();
    if ( ns.containsKey(key)) {
        value= ns[key];
    } else {
       set(key,defaultValue);
        value=defaultValue;
    }
    INFO(" %s.%s = %u ",_nameSpace.c_str(),key,value);
}

void Config::get(const char* key, double& value,
                 double defaultValue)
{
    load();
    JsonObject& ns=nameSpace();
    if ( ns.containsKey(key)) {
        value= ns[key];
    } else {
        set(key,defaultValue);
        value=defaultValue;
    }
    INFO(" %s.%s = %f ",_nameSpace.c_str(),key,value);
}


void Config::print(Str& str)
{
    load();
    
}

void Config::printPretty(Str& str)
{
    load();
}

Config config;
