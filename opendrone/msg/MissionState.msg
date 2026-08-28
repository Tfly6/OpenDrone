# Planner-owned mission state. The component that owns the mission queue must
# publish this message; trajectory adapters must never infer it from position,
# trajectory IDs, local references, or controller commands.
Header header

uint8 TYPE_SEQUENTIAL_GOAL = 1
uint8 TYPE_REFERENCE_PATH  = 2
uint8 mission_type

uint8 STATUS_ACTIVE    = 1
uint8 STATUS_SUCCEEDED = 2
uint8 STATUS_FAILED    = 3
uint8 status

# Monotonic native progress. For SEQUENTIAL_GOAL an item is one active goal
# consumed by the planner's own FSM. For REFERENCE_PATH the entire path is one
# item; intermediate Path poses are references and never become completion
# units. This message reports state and must not control goal advancement.
uint32 completed_items
uint32 total_items

# Optional planner-native explanation, especially for STATUS_FAILED.
string detail
