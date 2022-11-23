//
// Generated file, do not edit! Created by nedtool 5.6 from flooding/FloodingHeader.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include "FloodingHeader_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp


// forward
template<typename T, typename A>
std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec);

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// operator<< for std::vector<T>
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

FloodingHeader::FloodingHeader() : ::FieldsChunk()
{
    this->magicNo = 0;
    this->floodingVersion = 0;
    this->timeToLive = 0;
    this->seqno = 0;
}

FloodingHeader::FloodingHeader(const FloodingHeader& other) : ::FieldsChunk(other)
{
    copy(other);
}

FloodingHeader::~FloodingHeader()
{
}

FloodingHeader& FloodingHeader::operator=(const FloodingHeader& other)
{
    if (this==&other) return *this;
    ::FieldsChunk::operator=(other);
    copy(other);
    return *this;
}

void FloodingHeader::copy(const FloodingHeader& other)
{
    this->magicNo = other.magicNo;
    this->floodingVersion = other.floodingVersion;
    this->timeToLive = other.timeToLive;
    this->seqno = other.seqno;
    this->sourceId = other.sourceId;
}

void FloodingHeader::parsimPack(omnetpp::cCommBuffer *b) const
{
    doParsimPacking(b,(::FieldsChunk&)*this);
    doParsimPacking(b,this->magicNo);
    doParsimPacking(b,this->floodingVersion);
    doParsimPacking(b,this->timeToLive);
    doParsimPacking(b,this->seqno);
    doParsimPacking(b,this->sourceId);
}

void FloodingHeader::parsimUnpack(omnetpp::cCommBuffer *b)
{
    doParsimUnpacking(b,(::FieldsChunk&)*this);
    doParsimUnpacking(b,this->magicNo);
    doParsimUnpacking(b,this->floodingVersion);
    doParsimUnpacking(b,this->timeToLive);
    doParsimUnpacking(b,this->seqno);
    doParsimUnpacking(b,this->sourceId);
}

uint16_t FloodingHeader::getMagicNo() const
{
    return this->magicNo;
}

void FloodingHeader::setMagicNo(uint16_t magicNo)
{
    this->magicNo = magicNo;
}

uint16_t FloodingHeader::getFloodingVersion() const
{
    return this->floodingVersion;
}

void FloodingHeader::setFloodingVersion(uint16_t floodingVersion)
{
    this->floodingVersion = floodingVersion;
}

uint16_t FloodingHeader::getTimeToLive() const
{
    return this->timeToLive;
}

void FloodingHeader::setTimeToLive(uint16_t timeToLive)
{
    this->timeToLive = timeToLive;
}

uint32_t FloodingHeader::getSeqno() const
{
    return this->seqno;
}

void FloodingHeader::setSeqno(uint32_t seqno)
{
    this->seqno = seqno;
}

MacAddress& FloodingHeader::getSourceId()
{
    return this->sourceId;
}

void FloodingHeader::setSourceId(const MacAddress& sourceId)
{
    this->sourceId = sourceId;
}

class FloodingHeaderDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    FloodingHeaderDescriptor();
    virtual ~FloodingHeaderDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual const char *getFieldDynamicTypeString(void *object, int field, int i) const override;
    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(FloodingHeaderDescriptor)

FloodingHeaderDescriptor::FloodingHeaderDescriptor() : omnetpp::cClassDescriptor("FloodingHeader", "FieldsChunk")
{
    propertynames = nullptr;
}

FloodingHeaderDescriptor::~FloodingHeaderDescriptor()
{
    delete[] propertynames;
}

bool FloodingHeaderDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<FloodingHeader *>(obj)!=nullptr;
}

const char **FloodingHeaderDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *FloodingHeaderDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int FloodingHeaderDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 5+basedesc->getFieldCount() : 5;
}

unsigned int FloodingHeaderDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
    };
    return (field>=0 && field<5) ? fieldTypeFlags[field] : 0;
}

const char *FloodingHeaderDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "magicNo",
        "floodingVersion",
        "timeToLive",
        "seqno",
        "sourceId",
    };
    return (field>=0 && field<5) ? fieldNames[field] : nullptr;
}

int FloodingHeaderDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='m' && strcmp(fieldName, "magicNo")==0) return base+0;
    if (fieldName[0]=='f' && strcmp(fieldName, "floodingVersion")==0) return base+1;
    if (fieldName[0]=='t' && strcmp(fieldName, "timeToLive")==0) return base+2;
    if (fieldName[0]=='s' && strcmp(fieldName, "seqno")==0) return base+3;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceId")==0) return base+4;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *FloodingHeaderDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "uint16_t",
        "uint16_t",
        "uint16_t",
        "uint32_t",
        "MacAddress",
    };
    return (field>=0 && field<5) ? fieldTypeStrings[field] : nullptr;
}

const char **FloodingHeaderDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *FloodingHeaderDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int FloodingHeaderDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    FloodingHeader *pp = (FloodingHeader *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

const char *FloodingHeaderDescriptor::getFieldDynamicTypeString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldDynamicTypeString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    FloodingHeader *pp = (FloodingHeader *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string FloodingHeaderDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    FloodingHeader *pp = (FloodingHeader *)object; (void)pp;
    switch (field) {
        case 0: return ulong2string(pp->getMagicNo());
        case 1: return ulong2string(pp->getFloodingVersion());
        case 2: return ulong2string(pp->getTimeToLive());
        case 3: return ulong2string(pp->getSeqno());
        case 4: {std::stringstream out; out << pp->getSourceId(); return out.str();}
        default: return "";
    }
}

bool FloodingHeaderDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    FloodingHeader *pp = (FloodingHeader *)object; (void)pp;
    switch (field) {
        case 0: pp->setMagicNo(string2ulong(value)); return true;
        case 1: pp->setFloodingVersion(string2ulong(value)); return true;
        case 2: pp->setTimeToLive(string2ulong(value)); return true;
        case 3: pp->setSeqno(string2ulong(value)); return true;
        default: return false;
    }
}

const char *FloodingHeaderDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        case 4: return omnetpp::opp_typename(typeid(MacAddress));
        default: return nullptr;
    };
}

void *FloodingHeaderDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    FloodingHeader *pp = (FloodingHeader *)object; (void)pp;
    switch (field) {
        case 4: return (void *)(&pp->getSourceId()); break;
        default: return nullptr;
    }
}


