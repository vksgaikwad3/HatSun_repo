#ifndef serverConfg_h
#define serverConfg_h




 /***************** Thing Speak Settings ************************************/

    // Milk Chiller Details
    #define milkChiller               0
    #define milkChillerRelay          1
    #define energyMeter               2
    #define threephase_meterCh_1      3
    #define threephase_meterCh_2      4
    #define threephase_meterCh_3      5
    
    
    #define milkChillerKey         "012J5HBACJ2MHTDG"   //"SC1VWOBO8W7Y0QD0"    //"17XJCSK0JW8FYJMR"        // "XTUFVMDZYZLA0TB6"        //api Key for Milk Chiller Channel
    #define milkChillerRelayKey    "EPZ6JCZWVS28VTDG"   //"BDVYS2C0U8GYASEP"    //"VC84FR07YIHODCR9"        // "5CFI74KGNN3Z3CR1"        //api Key for Milk Chiller Relay Channel
    #define geyeserenergyMeterKey  "5Z216KB1XUB37ZDQ"   //"Y18VDVNTMB5JVH2R"   //"S8VX0HDOCRPLM8NR"        //"DIU0GB1C3XJQTOFY"        //api Key for Energy Meter Channel
    #define threephase_meterKey_1  "GMFVWCUZAIDO6GX4"   //"VI5LB6LO5K0KEM4F"   //"1LGS22RIAA1S9VRC"        // 3 Ph meter channel 1 
    #define threephase_meterKey_2  "FFWFUG5Z8QUB91VN"   //"U41OQOXVRU19O19T"   //"I8GB4GJMUXABGCN6"        // 3 ph meter channel 2
    #define threephase_meterKey_3  "O4SJ4Z06JABLPWRK"   //"EDY1WMP50OM28D7X"   //"D93WZ1PG6K6372XE"        // 3ph meter channel 3
    
    
    String channel_apiKey[] ={milkChillerKey, milkChillerRelayKey, geyeserenergyMeterKey,threephase_meterKey_1,threephase_meterKey_2,threephase_meterKey_3 };                    //write API key
    
    // Milk Chiller Relays Channel
    
    //const int Channel_ID2 =  184701;                       //Milk Chiller Relays ID 
    //String milkChillerRelays_apiKey ="5CFI74KGNN3Z3CR1";                    //write API key for Channel_ID2(Milk Chiller Relays Section)



 #endif
