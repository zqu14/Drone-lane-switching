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

cplusplus {{
#include <inet/common/packet/chunk/FieldsChunk.h>
#include <inet/linklayer/common/MacAddress.h>
typedef inet::FieldsChunk FieldsChunk;
typedef inet::MacAddress MacAddress;
}}

class noncobject FieldsChunk;
class noncobject MacAddress;


class FloodingHeader extends FieldsChunk
{
    uint16_t    magicNo;
    uint16_t    floodingVersion;
    uint16_t    timeToLive;    
    uint32_t    seqno;         // sequence number
    MacAddress  sourceId;      // Mac address of original source, used as sender id
}
