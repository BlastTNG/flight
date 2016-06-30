#include <CUnit/CUnit.h>
#include "CUnit/Basic.h"
#include <stdio.h>
#include "actual_cycles.c"

int tests_run = 0;

void test_generate() {
  HeatController* hc = generate_controller();
  CU_ASSERT_EQUAL(hc->t250_history[100], 0);
  CU_ASSERT_EQUAL(hc->mode, NORMAL);
  turn_off(hc);
}

void test_avg() {
  double arr[2];
  arr[0] = 0;
  arr[1] = 1;
  CU_ASSERT_DOUBLE_EQUAL(average(arr, 2), 0.5, 0.0001);
  arr[0] = 5;
  arr[1] = 7;
  CU_ASSERT_DOUBLE_EQUAL(average(arr, 2), 6, 0.001);
}

void test_too_hot() {
  int i;
  HeatController* hc = generate_controller();
  for (i = 0; i < 200; ++i) {
    hc->t250_history[i] = T_ARRAY_CRITICAL + 1;
  }
  CU_ASSERT(is_array_overheating(hc));
  
  for (i = 0; i < 200; ++i) {
    hc->t250_history[i] = T_ARRAY_CRITICAL - 0.1;
  }
  CU_ASSERT(!is_array_overheating(hc));
  turn_off(hc);
}

void normal_to_fridge_transition() {
  int i;
  HeatController* hc = generate_controller();
  for (i = 0; i < 200; ++i) {
    hc->t250_history[i] = T_ARRAY_CRITICAL + 0.1;
  }
  CU_ASSERT(is_array_overheating(hc));
  CU_ASSERT_EQUAL(hc->mode, NORMAL);
  read_triggers(hc);
  CU_ASSERT_EQUAL(hc->mode, FRIDGE_CYCLE);
  turn_off(hc);
}

void fridge_to_normal_charcoal_transition() {
  HeatController* hc = generate_controller();
  hc->mode = FRIDGE_CYCLE;
  hc->tcharcoal = T_CHARCOAL_CRITICAL + 0.1;
  read_triggers(hc);
  CU_ASSERT_EQUAL(hc->mode, NORMAL);
  turn_off(hc);
}

void fridge_to_normal_time_transition() {
  HeatController* hc = generate_controller();
  hc->mode = FRIDGE_CYCLE;
  hc->seconds_counter = 2400;
  read_triggers(hc);
  CU_ASSERT_EQUAL(hc->mode, NORMAL);
  turn_off(hc);
}

void full_cycle() {
  int i;
  HeatController* hc = generate_controller();
  CU_ASSERT_EQUAL(hc->mode, NORMAL);
  
  // Keep average <= T_ARRAY_CRITICAL
  for (i = 0; i < 300; ++i) {
    hc->t250 = T_ARRAY_CRITICAL - 0.001;
    hc->t350 = T_ARRAY_CRITICAL - 0.001;
    hc->t500 = T_ARRAY_CRITICAL - 0.001;
    record_history(hc);
    read_triggers(hc);
    CU_ASSERT_EQUAL(hc->mode, NORMAL);
  }

  // Transition to Fridge mode
  hc->t250 = T_ARRAY_CRITICAL + 1000;
  record_history(hc);
  read_triggers(hc);
  CU_ASSERT_EQUAL(hc->mode, FRIDGE_CYCLE); 
  
  // Transition to normal with the charcoal overheating
  hc->tcharcoal = T_CHARCOAL_CRITICAL - 1;
  for (i = 0; i < 2300; ++i) {
    read_triggers(hc);
    CU_ASSERT_EQUAL(hc->mode, FRIDGE_CYCLE);
  }
  
  hc->tcharcoal = T_CHARCOAL_CRITICAL + 1;
  read_triggers(hc);
  CU_ASSERT_EQUAL(hc->mode, NORMAL);

  // Transition back to FRIDGE mode
  read_triggers(hc);
  CU_ASSERT_EQUAL(hc->mode, FRIDGE_CYCLE);

  // Transition back to normal, this time with the timeout
  hc->tcharcoal = T_CHARCOAL_CRITICAL - 1;
  for (i = 0; i < 2399; ++i) {
    read_triggers(hc);
    CU_ASSERT_EQUAL(hc->mode, FRIDGE_CYCLE);
  }
  read_triggers(hc);
  CU_ASSERT_EQUAL(hc->mode, NORMAL);

  turn_off(hc);
}

/* The main() function for setting up and running the tests.
 * Returns a CUE_SUCCESS on successful running, another
 * CUnit error code on failure.
 */
int main()
{
   CU_pSuite pSuite = NULL;

   /* initialize the CUnit test registry, and add a suite */
   CU_initialize_registry();
   pSuite = CU_add_suite("Suite_1", NULL, NULL);

   /* add the tests to the suite */
   CU_add_test(pSuite, "generate", test_generate); 
   CU_add_test(pSuite, "average", test_avg);
   CU_add_test(pSuite, "too hot", test_too_hot);
   CU_add_test(pSuite, "normal to fridge transition", normal_to_fridge_transition);
   CU_add_test(pSuite, "fridge to normal transition - charcoal", fridge_to_normal_charcoal_transition);
   CU_add_test(pSuite, "fridge to normal transition - time", fridge_to_normal_time_transition);
   CU_add_test(pSuite, "complete cycle", full_cycle);
   
   /* Run all tests using the CUnit Basic interface */
   //CU_basic_set_mode(CU_BRM_VERBOSE);
   CU_basic_run_tests();
   CU_cleanup_registry();
   return CU_get_error();
}
