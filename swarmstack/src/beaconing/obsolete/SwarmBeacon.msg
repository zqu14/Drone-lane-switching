//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
// This is the message type that represents beacons. Beacons are
// transmitted periodically by each and every node to its neighbors
// (local broadcast)


cplusplus {{
#include <inet/common/geometry/common/Coord.h>
#include <inet/common/packet/chunk/FieldsChunk.h>
#include <inet/linklayer/common/MacAddress.h>
typedef inet::Coord Coord;
typedef inet::FieldsChunk FieldsChunk;
typedef inet::MacAddress MacAddress;
}}

class noncobject Coord;
class noncobject FieldsChunk;
class noncobject MacAddress;


class SwarmBeacon extends FieldsChunk
{
    uint16_t    magicNo;       // a magic number identifying beacons
    uint16_t    version;       // version of the SwarmStack protocol
    uint32_t    seqno;         // sequence number
    MacAddress  senderId;      // Mac address of sender, used as sender id

    Coord        currPosition;  // current position in given coordinate system
    Coord        currVelocity;  // current direction vector
}
