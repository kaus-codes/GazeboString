# Audit bookkeeping failure — run remains rejected

This development run stopped after eight verified placements. The next attached entity was part_033. The loader CLI timed out after issuing its creation request, and the old loader registered metadata only after a successful CLI return. The thread therefore exited without registering this entity.

The sorter selected RED, and the captured creation request in full-seed-11-console.log specifies red material for part_033. The old auditor grouped unknown metadata with wrong-color or association failures and incremented incorrect_picks. That legacy counter does not by itself establish a camera color error. This run remains invalid: the required independent audit evidence was missing and fourteen placements were not verified.

The corrected loader registers each uniquely named request before sending it, records uncertain outcomes, confirms actual poses separately, and survives CLI timeouts. A regression test exercises creation followed by timeout. Later runs distinguish missing audit metadata, wrong-color grasps, and identity association failures.
