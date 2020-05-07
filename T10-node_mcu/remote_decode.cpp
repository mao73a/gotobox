#include "remote_decode.h"

int remoteDecode(decode_results pDecodeResults)
{
    int vResult=REMOTE_UNKNOWN;
     
    if (pDecodeResults.decode_type == NEC)
    {
        switch(pDecodeResults.value){
            case 0xFF38C7: 
                vResult= REMOTE_OK; 
                break;
            case 0xFF6897: 
                vResult=REMOTE_ASTER;
                break;
            case 0xFFB04F: 
                vResult=REMOTE_HASH;    
                break;
                
            case 0xFF10EF: 
                vResult=REMOTE_LEFT;                
                break;
            case 0xFF5AA5: 
                vResult=REMOTE_RIGHT;                
                break;
            case 0xFF18E7: 
                vResult=REMOTE_UP;                
                break;
            case 0xFF4AB5: 
                vResult=REMOTE_DOWN;
                break;
                
            case 0xFF9867: 
                vResult=REMOTE_0;
                break;
            case 0xFFA25D: 
                vResult=REMOTE_1;
                break;
            case 0xFF629D: 
                vResult=REMOTE_2;
                break;
            case 0xFFE21D: 
                vResult=REMOTE_3;
                break;
            case 0xFF22DD: 
                vResult=REMOTE_4;
                break;
            case 0xFF02FD: 
                vResult=REMOTE_5;
                break;
            case 0xFFC23D: 
                vResult=REMOTE_6;
                break;
            case 0xFFE01F: 
                vResult=REMOTE_7;                
                break;
            case 0xFFA857: 
                vResult=REMOTE_8;
                break;
            case 0xFF906F: 
                vResult=REMOTE_9;
                break;
        }
    }


    return vResult;
}
