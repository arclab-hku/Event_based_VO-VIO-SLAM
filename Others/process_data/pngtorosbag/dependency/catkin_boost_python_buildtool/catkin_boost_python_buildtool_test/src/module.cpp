#include <boost/python.hpp>
using namespace boost::python;

int test() {
  return 42;
}

void exportTest() {
  using namespace boost::python;
  
  def("test", test, "test");
}

BOOST_PYTHON_MODULE(libcatkin_boost_python_test) {
  exportTest();
}
