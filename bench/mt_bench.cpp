#include <mockturtle/networks/aig.hpp>
#include <mockturtle/networks/buffered.hpp>
#include <mockturtle/io/verilog_reader.hpp>
#include <mockturtle/algorithms/aqfp/buffer_insertion.hpp>
#include <mockturtle/algorithms/aqfp/buffer_verification.hpp>
#include <lorina/verilog.hpp>
#include <mockturtle/io/write_verilog.hpp>
#include <iostream>
using namespace mockturtle;
int main(int argc, char** argv) {
  aig_network aig;
  if (lorina::read_verilog(argv[1], verilog_reader(aig)) != lorina::return_code::success) { std::cerr << "read fail\n"; return 1; }
  if (argc > 3) write_verilog(aig, argv[3]);
  uint32_t cap = argc > 2 ? std::stoi(argv[2]) : 4;
  std::cout << "{\"gates\":" << aig.num_gates();
  struct Mode { const char* name; buffer_insertion_params::scheduling_policy s; int opt; };
  Mode modes[] = {
    {"ASAP_depth", buffer_insertion_params::ASAP_depth, 0},
    {"ALAP_depth", buffer_insertion_params::ALAP_depth, 0},
    {"better_depth", buffer_insertion_params::better_depth, 0},
    {"better_depth+until_sat", buffer_insertion_params::better_depth, 2},
  };
  for (auto& m : modes) {
    buffer_insertion_params ps;
    ps.assume.balance_cios = true;
    ps.assume.ci_capacity = 1;
    ps.assume.splitter_capacity = cap;
    ps.assume.num_phases = 1;
    ps.scheduling = m.s;
    if (m.opt == 1) ps.optimization_effort = buffer_insertion_params::one_pass;
    else if (m.opt == 2) ps.optimization_effort = buffer_insertion_params::until_sat;
    buffer_insertion<aig_network> bi(aig, ps);
    buffered_aig_network baig;
    uint32_t nb = bi.run(baig);
    bool ok = verify_aqfp_buffer(baig, ps.assume, bi.pi_levels());
    std::cout << ",\"" << m.name << "\":{\"buffers\":" << nb << ",\"depth\":" << bi.depth() << ",\"ok\":" << (ok ? "true" : "false") << "}";
  }
  std::cout << "}\n";
}
