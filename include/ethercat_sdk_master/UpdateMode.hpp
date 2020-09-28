#pragma once

namespace ecat_master{

/*!
 * Update Mode.
 * - NonStandalone:
 *   Communication update timing is handled externally.
 * - StandaloneEnforceRate:
 *   Create the necessary timeout such that the average update rate corresponds to
 *   target rate (compensate for updates that took too long).
 * - StandaloneEnforceStep:
 *   Create the necessary timeout such that the update time step correspnds to
 *   the target time step. No compensation for updates that took too long.
 */
enum class UpdateMode {NonStandalone, StandaloneEnforceRate, StandaloneEnforceStep};
}
