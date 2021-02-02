/*!
*****************************************************************************
*  
* \file sUDP.h      
* \version   2.6
* \date Feb 27, 2013
* \author Sebastian Schwab
*****************************************************************************
* \brief Module for UDP communication
* \details
* The Module sUDP supports the usage of UDP communications within 
* the CarMaker. Therefor it provides functions for sending and receiving 
* UDP messages. These functions are architecture independend. Therefor  
* the same code can be used for several architectures.
* 
* The module supports only the sending an receiving of UDP messages. There is
* no protocol specified.  
*
* \cond copyright  
* (C) IPG Automotive GmbH                                  
* Bannwaldallee 60             Phone  +49.721.98520.0  
* 76185 Karlsruhe              Fax    +49.721.98520.99 
* Germany                      WWW    www.ipg-automotive.com \endcond
***************************************************************************** 
*/


#ifndef _SIMPLE_UDP__H_
#define _SIMPLE_UDP__H_


typedef void (* tUDP_ReadFnc) (void *msg, int len, unsigned short int srcPort);

/*! \fn int UDP_New_InSocket (unsigned short inPort, tUDP_ReadFnc ReadFnc)
 * \brief Creates an UDP Socket
 * \details Does something really cool.
 * \param inPort the port given in.
 * \param ReadFnc whatever.
 * \returns SockID  \n >=0 -> SockID (can be reused for socket configurations like setsockopt() \n
 *                     -1 = not ok.
 */
int UDP_New_InSocket (unsigned short inPort, tUDP_ReadFnc ReadFnc);
/*!
 * \brief Creates an UDP tudpOutSocket pointer
 * \details Does something really cool.
 * \param serverName servername input.
 * \param outPort Output Port.
 * \returns SockID
 */
int UDP_New_OutSocket(char *serverName, unsigned short outPort);
/*!
 * \brief Creates an UDP tudpOutSocket pointer, using the opened socket from UDP_New_InSocket
 * \details Does something really cool.
 * \param serverName servername input.
 * \param outPort Output Port.
 * \param sockIn SockID from UDP_New_InSocket
 * \returns SockID
 */
int UDP_Use_InSocket_as_OutSocket (char *serverName, unsigned short outPort, int sockIn);


int UDP_In(void);
int UDP_SendMsg (int SockID, void *buf, int len);
int UDP_ClearInSockets (int SockId);
int UDP_Delete(void);
int UDP_Cleanup (void);

#endif /* _SIMPLE_UDP__H_ */
