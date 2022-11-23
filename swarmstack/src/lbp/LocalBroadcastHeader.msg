//
//

cplusplus {{
#include <inet/common/packet/chunk/FieldsChunk.h>
#include <inet/linklayer/common/MacAddress.h>
typedef inet::FieldsChunk FieldsChunk;
typedef inet::MacAddress MacAddress;
}}

class noncobject FieldsChunk;
class noncobject MacAddress;


class LocalBroadcastHeader extends FieldsChunk
{
    uint16_t    magicNo;       // a magic number identifying local broadcasts
    uint16_t    version;       // version of the local broadcast protocol
    uint32_t    seqno;         // sequence number
    MacAddress  senderId;      // Mac address of sender, used as sender id
}
