#include "api_bridge.hpp"
#include <mvkit/cpp/mv_dict.hpp>
#include <mvkit/c/mv_dict.h>
#include <utility>

namespace mv {
void CreateDict(MvTuple* dict) {
  cpp_api::require_output(dict, "CreateDict");
  ::MvTuple h=nullptr;
  cpp_api::check_status(mv_create_dict(&h), "CreateDict");
  *dict=detail::MvTupleAccess::Adopt(h);
}
void SetDictTuple(const MvTuple& dict,const MvTuple& key,const MvTuple& value) {
  cpp_api::check_status(mv_set_dict_tuple(dict.ControlInput(),key.ControlInput(),value.ControlInput()), "SetDictTuple");
}
void GetDictTuple(const MvTuple& dict,const MvTuple& key,MvTuple* value) {
  cpp_api::require_output(value, "GetDictTuple");
  ::MvTuple h=nullptr;
  cpp_api::check_status(mv_get_dict_tuple(dict.ControlInput(),key.ControlInput(),&h), "GetDictTuple");
  *value=detail::MvTupleAccess::Adopt(h);
}
void SetDictObject(const MvObject& object,const MvTuple& dict,const MvTuple& key) {
  cpp_api::check_status(mv_set_dict_object(object.NativeHandle(),dict.ControlInput(),key.ControlInput()), "SetDictObject");
}
void GetDictObject(MvObject* object,const MvTuple& dict,const MvTuple& key) {
  cpp_api::require_output(object, "GetDictObject");
  ::MvObject h=nullptr;
  cpp_api::check_status(mv_get_dict_object(&h,dict.ControlInput(),key.ControlInput()), "GetDictObject");
  detail::MvObjectAccess::Commit(object,detail::MvObjectAccess::Adopt(h));
}
MvDict::MvDict() { CreateDict(); }
MvDict::MvDict(const MvDict&) = default;
MvDict::MvDict(MvDict&&) noexcept = default;
MvDict& MvDict::operator=(const MvDict&) = default;
MvDict& MvDict::operator=(MvDict&&) noexcept = default;
MvDict::~MvDict() noexcept = default;
MvDict::MvDict(const MvHandle& handle) : MvHandle(handle) {
  if (Type()!=MvObjectType::Dict) throw MvException(MvStatus::TypeMismatch,MvErrorCode::InvalidType,
      "MvDict","handle",-1,"control handle is not a dictionary");
}
void MvDict::CreateDict() {
  MvTuple result;mv::CreateDict(&result);
  MvHandle::operator=(result.H());
}
void MvDict::SetDictTuple(const MvTuple& key,const MvTuple& value) const { mv::SetDictTuple(MvTuple(*this),key,value); }
void MvDict::SetDictTuple(const MvString& key,const MvTuple& value) const { SetDictTuple(MvTuple(key),value); }
void MvDict::SetDictTuple(const char* key,const MvTuple& value) const { SetDictTuple(MvTuple(key),value); }
MvTuple MvDict::GetDictTuple(const MvTuple& key) const { MvTuple result;mv::GetDictTuple(MvTuple(*this),key,&result);return result; }
MvTuple MvDict::GetDictTuple(const MvString& key) const { return GetDictTuple(MvTuple(key)); }
MvTuple MvDict::GetDictTuple(const char* key) const { return GetDictTuple(MvTuple(key)); }
void MvDict::SetDictObject(const MvObject& object,const MvTuple& key) const { mv::SetDictObject(object,MvTuple(*this),key); }
void MvDict::SetDictObject(const MvObject& object,const MvString& key) const { SetDictObject(object,MvTuple(key)); }
void MvDict::SetDictObject(const MvObject& object,const char* key) const { SetDictObject(object,MvTuple(key)); }
MvObject MvDict::GetDictObject(const MvTuple& key) const { MvObject result;mv::GetDictObject(&result,MvTuple(*this),key);return result; }
MvObject MvDict::GetDictObject(const MvString& key) const { return GetDictObject(MvTuple(key)); }
MvObject MvDict::GetDictObject(const char* key) const { return GetDictObject(MvTuple(key)); }
}

namespace mv {
void GetDictParam(const MvTuple& dict,const MvTuple& name,const MvTuple& keys,MvTuple* result) {
  cpp_api::TupleOutputs<1> out("GetDictParam",{result});
  cpp_api::check_status(mv_get_dict_param(dict.ControlInput(),name.ControlInput(),keys.ControlInput(),out.Out(0)),"GetDictParam");out.Commit();
}
void RemoveDictKey(const MvTuple& dict,const MvTuple& keys) {
  cpp_api::check_status(mv_remove_dict_key(dict.ControlInput(),keys.ControlInput()),"RemoveDictKey");
}
}
