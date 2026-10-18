
#include <exception>
#include <mvkit/cpp/mv_system.hpp>

#include "api_bridge.hpp"

namespace mv {

void SetSystem(const MvTuple& SystemParameter, const MvTuple& Value) {
  cpp_api::check_status(mv_set_system(SystemParameter.ControlInput(), Value.ControlInput()), "SetSystem");
}
void GetSystem(const MvTuple& Query, MvTuple* Information) {
  cpp_api::TupleOutputs<1> output("GetSystem", {Information});
  cpp_api::check_status(mv_get_system(Query.ControlInput(), output.Out(0)), "GetSystem");
  output.Commit();
}
void MvSystem::SetSystem(const MvTuple& name, const MvTuple& value) { mv::SetSystem(name, value); }
void MvSystem::SetSystem(const MvString& name, const MvString& value) { mv::SetSystem(name, value); }
void MvSystem::SetSystem(const char* name, const char* value) { mv::SetSystem(name, value); }
void MvSystem::SetSystem(const wchar_t* name, const wchar_t* value) {
  mv::SetSystem(MvString(name), MvString(value));
}
MvTuple MvSystem::GetSystem(const MvTuple& query) {
  MvTuple out;
  mv::GetSystem(query, &out);
  return out;
}
MvTuple MvSystem::GetSystem(const MvString& query) { return GetSystem(MvTuple(query)); }
MvTuple MvSystem::GetSystem(const char* query) { return GetSystem(MvTuple(query)); }
MvTuple MvSystem::GetSystem(const wchar_t* query) { return GetSystem(MvTuple(MvString(query))); }

}  // namespace mv
