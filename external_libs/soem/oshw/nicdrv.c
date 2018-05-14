/*
 * Simple Open EtherCAT Master Library
 *
 * File    : nicdrv.c
 * Version : 1.3.1
 * Date    : 11-03-2015
 * Copyright (C) 2005-2015 Speciaal Machinefabriek Ketels v.o.f.
 * Copyright (C) 2005-2015 Arthur Ketels
 * Copyright (C) 2008-2009 TU/e Technische Universiteit Eindhoven
 * Copyright (C) 2014-2015 rt-labs AB , Sweden
 *
 * SOEM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the Free
 * Software Foundation.
 *
 * SOEM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * As a special exception, if other files instantiate templates or use macros
 * or inline functions from this file, or you compile this file and link it
 * with other works to produce a work based on this file, this file does not
 * by itself cause the resulting work to be covered by the GNU General Public
 * License. However the source code for this file must still be made available
 * in accordance with section (3) of the GNU General Public License.
 *
 * This exception does not invalidate any other reasons why a work based on
 * this file might be covered by the GNU General Public License.
 *
 * The EtherCAT Technology, the trade name and logo “EtherCAT” are the intellectual
 * property of, and protected by Beckhoff Automation GmbH. You can use SOEM for
 * the sole purpose of creating, using and/or selling or otherwise distributing
 * an EtherCAT network master provided that an EtherCAT Master License is obtained
 * from Beckhoff Automation GmbH.
 *
 * In case you did not receive a copy of the EtherCAT Master License along with
 * SOEM write to Beckhoff Automation GmbH, Eiserstraße 5, D-33415 Verl, Germany
 * (www.beckhoff.com).
 */

/** \file
 * \brief
 * EtherCAT RAW socket driver.
 *
 * Low level interface functions to send and receive EtherCAT packets.
 * EtherCAT has the property that packets are only send by the master,
 * and the send packets allways return in the receive buffer.
 * There can be multiple packets "on the wire" before they return.
 * To combine the received packets with the original send packets a buffer
 * system is installed. The identifier is put in the index item of the
 * EtherCAT header. The index is stored and compared when a frame is received.
 * If there is a match the packet can be combined with the transmit packet
 * and returned to the higher level function.
 *
 * The socket layer can exhibit a reversal in the packet order (rare).
 * If the Tx order is A-B-C the return order could be A-C-B. The indexed buffer
 * will reorder the packets automatically.
 *
 * The "redundant" option will configure two sockets and two NIC interfaces.
 * Slaves are connected to both interfaces, one on the IN port and one on the
 * OUT port. Packets are send via both interfaces. Any one of the connections
 * (also an interconnect) can be removed and the slaves are still serviced with
 * packets. The software layer will detect the possible failure modes and
 * compensate. If needed the packets from interface A are resend through interface B.
 * This layer if fully transparent for the higher layers.
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h> 
#include <sys/socket.h> 
#include <unistd.h>
#include <sys/time.h> 
#include <time.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <netpacket/packet.h>
#include <pthread.h>

#include "oshw.h"
#include "osal.h"

/** Redundancy modes */
enum
{
   /** No redundancy, single NIC mode */
   ECT_RED_NONE,
   /** Double redundant NIC connecetion */
   ECT_RED_DOUBLE
};


/** Primary source MAC address used for EtherCAT.
 * This address is not the MAC address used from the NIC.
 * EtherCAT does not care about MAC addressing, but it is used here to
 * differentiate the route the packet traverses through the EtherCAT
 * segment. This is needed to find out the packet flow in redundant
 * configurations. */
uint16 priMAC[3] = { 0x782b, 0xcb9d, 0x70c5 };

/** second MAC word is used for identification */
#define RX_PRIM priMAC[1]
/** second MAC word is used for identification */
#define RX_SEC secMAC[1]

static void ecx_clear_rxbufstat(int *rxbufstat)
{
   int i;
   for(i = 0; i < EC_MAXBUF; i++)
   {
      rxbufstat[i] = EC_BUF_EMPTY;
   }
}

/** Basic setup to connect NIC to socket.
 * @param[in] port        = port context struct
 * @param[in] ifname      = Name of NIC device, f.e. "eth0"
 * @return >0 if succeeded
 */
int ecx_setupnic(ecx_portt *port, const char *ifname)
{
   int i;
   int r, rval, ifindex;
   struct timeval timeout;
   struct ifreq ifr;
   struct sockaddr_ll sll = {0};
   int *psock;

   rval = 0;
    pthread_mutex_init(&(port->getindex_mutex), NULL);
    pthread_mutex_init(&(port->tx_mutex)      , NULL);
    pthread_mutex_init(&(port->rx_mutex)      , NULL);
    port->sockhandle        = -1;
    port->lastidx           = 0;
    port->stack.sock        = &(port->sockhandle);
    port->stack.txbuf       = &(port->txbuf);
    port->stack.txbuflength = &(port->txbuflength);
    port->stack.tempbuf     = &(port->tempinbuf);
    port->stack.rxbuf       = &(port->rxbuf);
    port->stack.rxbufstat   = &(port->rxbufstat);
    ecx_clear_rxbufstat(&(port->rxbufstat[0]));
    psock = &(port->sockhandle);

   /* we use RAW packet socket, with packet type ETH_P_ECAT */
   *psock = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ECAT));
   
   timeout.tv_sec =  0;
   timeout.tv_usec = 1; 
   r = setsockopt(*psock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
   r = setsockopt(*psock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
   i = 1;
   r = setsockopt(*psock, SOL_SOCKET, SO_DONTROUTE, &i, sizeof(i));
   /* connect socket to NIC by name */
   strcpy(ifr.ifr_name, ifname);
   r = ioctl(*psock, SIOCGIFINDEX, &ifr);
   ifindex = ifr.ifr_ifindex;
   strcpy(ifr.ifr_name, ifname);
   ifr.ifr_flags = 0;
   /* reset flags of NIC interface */
   r = ioctl(*psock, SIOCGIFFLAGS, &ifr);
   /* set flags of NIC interface, here promiscuous and broadcast */
   ifr.ifr_flags = ifr.ifr_flags | IFF_PROMISC | IFF_BROADCAST;
   r = ioctl(*psock, SIOCGIFFLAGS, &ifr);
   /* bind socket to protocol, in this case RAW EtherCAT */
   sll.sll_family = AF_PACKET;
   sll.sll_ifindex = ifindex;
   sll.sll_protocol = htons(ETH_P_ECAT);
   r = bind(*psock, (struct sockaddr *)&sll, sizeof(sll));

   memcpy(priMAC, ifr.ifr_hwaddr.sa_data, 6);
   port->rxsa = htons(priMAC[2]);
   /* setup ethernet headers in tx buffers so we don't have to repeat it */
   for (i = 0; i < EC_MAXBUF; i++) 
   {
      ec_setupheader(&(port->txbuf[i]), priMAC);
      port->rxbufstat[i] = EC_BUF_EMPTY;
   }
   ec_setupheader(&(port->txbuf2), priMAC);
   if (r == 0) rval = 1;
   
   return rval;
}

/** Close sockets used
 * @param[in] port        = port context struct
 * @return 0
 */
int ecx_closenic(ecx_portt *port) 
{
   if (port->sockhandle >= 0) 
      close(port->sockhandle);
   
   return 0;
}

/** Fill buffer with ethernet header structure.
 * Destination MAC is allways broadcast.
 * Ethertype is allways ETH_P_ECAT.
 * @param[out] p = buffer
 */
void ec_setupheader(void *p, uint16_t *mac)
{
   ec_etherheadert *bp;
   bp = p;
   bp->da0 = htons(0xffff);
   bp->da1 = htons(0xffff);
   bp->da2 = htons(0xffff);
   bp->sa0 = htons(mac[0]);
   bp->sa1 = htons(mac[1]);
   bp->sa2 = htons(mac[2]);
   bp->etype = htons(ETH_P_ECAT);
}

/** Get new frame identifier index and allocate corresponding rx buffer.
 * @param[in] port        = port context struct
 * @return new index.
 */
int ecx_getindex(ecx_portt *port)
{
   int idx;
   int cnt;

   pthread_mutex_lock( &(port->getindex_mutex) );

   idx = port->lastidx + 1;
   /* index can't be larger than buffer array */
   if (idx >= EC_MAXBUF) 
   {
      idx = 0;
   }
   cnt = 0;
   /* try to find unused index */
   while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF))
   {
      idx++;
      cnt++;
      if (idx >= EC_MAXBUF) 
      {
         idx = 0;
      }
   }
   port->rxbufstat[idx] = EC_BUF_ALLOC;
   port->lastidx = idx;

   pthread_mutex_unlock( &(port->getindex_mutex) );
   
   return idx;
}

/** Set rx buffer status.
 * @param[in] port        = port context struct
 * @param[in] idx      = index in buffer array
 * @param[in] bufstat  = status to set
 */
void ecx_setbufstat(ecx_portt *port, int idx, int bufstat)
{
   port->rxbufstat[idx] = bufstat;
}

/** Transmit buffer over socket (non blocking).
 * @param[in] port        = port context struct
 * @param[in] idx = index in tx buffer array
 * @return socket send result
 */
int ecx_outframe(ecx_portt *port, int idx)
{
   ec_etherheadert *ehp;
   int lp, rval;
   ec_stackT *stack;

   ehp = (ec_etherheadert *)&(port->txbuf[idx]);
   /* rewrite MAC source address 1 to primary */
   ehp->sa1 = htons(priMAC[1]);
   /* transmit over primary socket*/
   stack = &(port->stack);

   lp = (*stack->txbuflength)[idx];
   rval = send(*stack->sock, (*stack->txbuf)[idx], lp, 0);
   (*stack->rxbufstat)[idx] = EC_BUF_TX;
   
   return rval;
}

/** Non blocking read of socket. Put frame in temporary buffer.
 * @param[in] port        = port context struct
 * @param[in] stacknumber = 0=primary 1=secondary stack
 * @return >0 if frame is available and read
 */
static int ecx_recvpkt(ecx_portt *port)
{
   int lp, bytesrx;
   ec_stackT *stack;

   stack = &(port->stack);

   lp = sizeof(port->tempinbuf);
   bytesrx = recv(*stack->sock, (*stack->tempbuf), lp, 0);
   port->tempinbufs = bytesrx;
   
   return (bytesrx > 0);
}

/** Non blocking receive frame function. Uses RX buffer and index to combine
 * read frame with transmitted frame. To compensate for received frames that
 * are out-of-order all frames are stored in their respective indexed buffer.
 * If a frame was placed in the buffer previously, the function retreives it
 * from that buffer index without calling ec_recvpkt. If the requested index
 * is not already in the buffer it calls ec_recvpkt to fetch it. There are
 * three options now, 1 no frame read, so exit. 2 frame read but other
 * than requested index, store in buffer and exit. 3 frame read with matching
 * index, store in buffer, set completed flag in buffer status and exit.
 * 
 * @param[in] port        = port context struct
 * @param[in] idx         = requested index of frame
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME or EC_OTHERFRAME.
 */
int ecx_inframe(ecx_portt *port, int idx)
{
   uint16  l;
   int     rval;
   int     idxf;
   ec_etherheadert *ehp;
   ec_comt *ecp;
   ec_stackT *stack;
   ec_bufT *rxbuf;


   stack = &(port->stack);
   rval = EC_NOFRAME;
   rxbuf = &(*stack->rxbuf)[idx];
   /* check if requested index is already in buffer ? */
   if ((idx < EC_MAXBUF) && ((*stack->rxbufstat)[idx] == EC_BUF_RCVD)) 
   {
      l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
      /* return WKC */
      rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
      /* mark as completed */
      (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
   }
   else 
   {
      pthread_mutex_lock(&(port->rx_mutex));
      /* non blocking call to retrieve frame from socket */
      if (ecx_recvpkt(port))
      {
         rval = EC_OTHERFRAME;
         ehp =(ec_etherheadert*)(stack->tempbuf);
         /* check if it is an EtherCAT frame */
         if (ehp->etype == htons(ETH_P_ECAT) && (ehp->sa2 == port->rxsa))
         {
            ecp =(ec_comt*)(&(*stack->tempbuf)[ETH_HEADERSIZE]); 
            l = etohs(ecp->elength) & 0x0fff;
            idxf = ecp->index;
            /* found index equals requested index ? */
            if (idxf == idx)
            {
               /* yes, put it in the buffer array (strip Ethernet header) */
               memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
               /* return WKC */
               rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
               /* mark as completed */
               (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
            }
            else 
            {
               /* check if index exist? */
               if (idxf < EC_MAXBUF)
               {
                  rxbuf = &(*stack->rxbuf)[idxf];
                  /* put it in the buffer array (strip Ethernet header) */
                  memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE], (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
                  /* mark as received */
                  (*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
               }
               else 
               {
                  /* strange things happened */
               }
            }
         }
      }
      pthread_mutex_unlock( &(port->rx_mutex) );
      
   }
   
   /* WKC if mathing frame found */
   return rval;
}

/** Blocking receive frame function.
 * @param[in] port        = port context struct
 * @param[in] idx       = requested index of frame
 * @param[in] timeout   = timeout in us
 * @return Workcounter if a frame is found with corresponding index, otherwise
 * EC_NOFRAME.
 */
int ecx_waitinframe(ecx_portt *port, int idx, int timeout)
{
   int wkc = EC_NOFRAME;
   osal_timert timer;
   
   osal_timer_start (&timer, timeout);
   do 
   {
      /* only read frame if not already in */
      if (wkc <= EC_NOFRAME)
         wkc  = ecx_inframe(port, idx);
   /* wait for both frames to arrive or timeout */   
   } while ((wkc <= EC_NOFRAME) && !osal_timer_is_expired(&timer));
   /* if nothing received, clear buffer index status so it can be used again */
   if (wkc <= EC_NOFRAME) 
   {
      ecx_setbufstat(port, idx, EC_BUF_EMPTY);
   }
   
   return wkc;
}

/** Blocking send and receive frame function. Used for non processdata frames.
 * A datagram is build into a frame and transmitted via this function. It waits
 * for an answer and returns the workcounter. The function retries if time is
 * left and the result is WKC=0 or no frame received.
 *
 * The function calls ec_outframe_red() and ec_waitinframe_red().
 *
 * @param[in] port        = port context struct
 * @param[in] idx      = index of frame
 * @param[in] timeout  = timeout in us
 * @return Workcounter or EC_NOFRAME
 */
int ecx_srconfirm(ecx_portt *port, int idx, int timeout)
{
   int wkc = EC_NOFRAME;
   osal_timert timer1;
   int recv_timeout = (timeout > EC_TIMEOUTRET)?timeout:EC_TIMEOUTRET;

   osal_timer_start (&timer1, timeout);
   do 
   {
      /* tx frame on primary */
      ecx_outframe(port, idx);
      /* get frame from primary */
      wkc = ecx_waitinframe(port, idx, recv_timeout);
   /* wait for answer with WKC>=0 or otherwise retry until timeout */   
   } while ((wkc <= EC_NOFRAME) && !osal_timer_is_expired (&timer1));
   /* if nothing received, clear buffer index status so it can be used again */
   if (wkc <= EC_NOFRAME) 
   {
      ecx_setbufstat(port, idx, EC_BUF_EMPTY);
   }
   
   return wkc;
}

#ifdef EC_VER1
int ec_setupnic(const char *ifname)
{
   return ecx_setupnic(&ecx_port, ifname);
}

int ec_closenic(void)
{
   return ecx_closenic(&ecx_port);
}

int ec_getindex(void)
{
   return ecx_getindex(&ecx_port);
}

void ec_setbufstat(int idx, int bufstat)
{
   ecx_setbufstat(&ecx_port, idx, bufstat);
}

int ec_outframe(int idx)
{
   return ecx_outframe(&ecx_port, idx);
}

int ec_inframe(int idx)
{
   return ecx_inframe(&ecx_port, idx);
}

int ec_waitinframe(int idx, int timeout)
{
   return ecx_waitinframe(&ecx_port, idx, timeout);
}

int ec_srconfirm(int idx, int timeout)
{
   return ecx_srconfirm(&ecx_port, idx, timeout);
}
#endif
