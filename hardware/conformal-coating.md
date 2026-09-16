# Conformal coating for the active-probe buffer stage

Scope: the small active-probe buffer board (op-amp follower + passives on a
flex PCB tab, see `active-probe-opamp-selection` / `active-probe-flex-design`)
that will live in or right next to the humid zone, close to actively growing
mycelium. This note is about protecting the **buffer IC, passives, and
traces** on that tab -- **not** the electrode sensing contact itself. Coating
the electrode tip would insulate it from the substrate and defeat the whole
point of an electrode.

Grounded in loci investigation `mycomidi-adc-project`, tag `conformal-coating`.

## The 4 main conformal coating families

| Family | Standard hobbyist products | Moisture/humidity resistance | Fungal/microbial resistance claim | Dielectric properties | Rework/repair difficulty |
|---|---|---|---|---|---|
| **Acrylic (AR)** | MG Chemicals 419D/419E (liquid, spray, pen), Kester ADEZ pen | Good; resists moisture, condensation, salt spray | Rated fungus-resistant (IPC-CC-830C qualification on 419E); attacked by acetone/alcohols in service | High dielectric strength, high insulation resistance | **Easiest.** Dissolves in solvents (acetone, MEK, dedicated strippers e.g. MG 8309); can also be scraped/peeled locally for spot rework |
| **Silicone (SR)** | MG Chemicals 422B (liquid, spray), silicone/acrylic hybrid 422C | Good, and best of the three "wet" coatings for wide thermal cycling + flexibility -- "minimum stress on components during thermal cycling" | Also marketed with the same fungus/moisture/corrosion resistance claim as acrylic and urethane | Good dielectric strength; softer, more flexible film than acrylic | **Easy-ish.** Elastomeric film can often be mechanically peeled/cut away locally after softening (heat or solvent swell), or removed with a silicone-specific stripper; more resistant to common solvents than acrylic, so slightly more work than acrylic but still hobbyist-tractable |
| **Urethane (UR)** | MG Chemicals 4223F (liquid, spray) | Good; positioned as the most chemically resistant of the three wet coatings | Same fungus/moisture/corrosion claim as acrylic/silicone | High dielectric strength, good abrasion resistance | **Hardest of the three wet coatings.** Explicitly marketed as more chemically resistant -- that resistance cuts both ways: it needs more aggressive solvent/longer soak to strip for rework, and can still damage substrate/silkscreen if forced |
| **Parylene (XY)** | Applied by a vendor via CVD (SCS Coatings, etc.) -- not a product you buy and apply yourself | **Best of the four.** Ultra-thin, pinhole-free, vapor-deposited film with excellent moisture/dielectric barrier and full coverage into crevices | Not typically marketed with a fungus-resistance claim the way the wet coatings are (SCS's antimicrobial "microRESIST" variant is a separate specialty/medical product with an added biocide, not standard parylene C) | Excellent (low dielectric constant, stable across frequency) | **Not hobbyist-reworkable at all.** Parylene C is insoluble in any solvent at room temperature; industrial rework requires mechanical micro-abrasion/media-blasting, oxygen-plasma etching, or laser ablation -- specialized equipment, not a bench operation |

Sources: MG Chemicals product pages for 419D/419E (acrylic), 422B/422C
(silicone), 4223F (urethane), and 8309 (remover), fetched live 2026-09-16;
SCS Coatings "Key Properties of SCS Parylene Coatings" page; Wikipedia
"Parylene" and "Conformal coating" articles (application methods, parylene
insolubility/rework limitation).

**Notable and slightly counter-intuitive finding:** all three "wet" hobbyist
coating families (acrylic, silicone, urethane) carry essentially the same
marketing claim -- "protects against moisture, condensation, humidity,
corrosion, **fungus**, dirt, dust, thermal shock..." -- word for word similar
across MG Chemicals' acrylic, silicone, and urethane product pages. Fungus
resistance is not a differentiator among the three wet families; it's
industry table stakes for any qualified conformal coating (tied to
IPC-CC-830C / the old MIL-I-46058C fungus-resistance test, which is really
about the cured film not feeding mold spores in storage/field humidity, see
gap note below). The real differentiators for this project are rework
difficulty and application method, not fungal resistance.

## Dielectric properties vs. the sensing contact -- explicit flag

All four families are good electrical insulators, which is exactly why none
of them should ever be allowed to reach the electrode's actual sensing
contact (the exposed pin tip described in `hardware/pin-board.md`, or the
buffer stage's own input pad where the electrode wire lands). If a coating
skins over the contact:

- It insulates the metal-to-substrate interface, blocking the ionic/ohmic
  contact the electrode depends on to pick up the bioelectric signal, i.e. it
  silently kills the channel rather than damaging anything.
- This applies to acrylic/silicone/urethane equally (all are dielectrics by
  design) and doubles for parylene, which is explicitly a vapor-deposited
  *conformal* coating that will coat every exposed surface in a CVD chamber,
  including a contact tip, unless it is masked first.

**Masking requirement:** mask the electrode's exposed sensing tip and any
connector contact pins before coating, and pull the mask immediately after
cure. For brush/pen/spray/dip application this is normal practice already
called out in the general conformal-coating literature ("users may need to
apply special masking to certain electronic components... masking must occur
over male contact pins... removing coating in keep-out areas is simple with
use of any solvent" -- MG Chemicals 419E product page). For a vendor-run
parylene CVD job, masking has to be specified explicitly to the vendor (e.g.
with a temporary latex/RTV dam or vendor-supplied masking tape rated for the
process) since there is no local person doing a brush stroke to skip the tip.

Scope for this buffer-stage board specifically: coat the op-amp package, its
decoupling/bias passives, and the surrounding traces; leave the electrode pad
and the header/connector pins that plug into the shielded cable to the
ADS131M08 module uncoated (or coated then re-exposed by pulling the mask).

## Hobbyist-accessible application methods

- **Brush-on pens** (MG Chemicals 419D acrylic pen, Kester ADEZ pen): best
  fit for this board. A small flex-tab buffer stage is exactly the
  "touch-up/localized coating" use case pens are built for -- precise,
  low-volume, no spray booth or dip tank needed, easy to keep off the
  electrode pad and connector pins with a steady hand or a bit of masking
  tape. MG's own pen instructions describe shaking the internal mixing ball
  and pressing the tip to prime flow, consistent with a controlled,
  localized dab-and-drag application rather than a flood coat.
- **Spray cans** (MG Chemicals 419D-340G acrylic, 422B-340G silicone,
  4223F-312G urethane): workable, but overkill and messier for a single
  small flex tab -- good masking discipline is harder with an aerosol than a
  pen, and overspray onto the electrode pad or nearby cabling is a real risk
  in a home/bench setting.
- **Dip:** the highest-volume, most-complete-coverage method in industry, but
  a poor fit here: dipping puts coating everywhere, including under
  components and into the electrode contact area and connector, and requires
  designing the board around dip masking (or accepting you'll have to strip
  and re-clear the contact afterward every time). Not recommended for a
  one-off hobbyist flex tab.

**Recommendation for this board: pen application (MG 419D or Kester ADEZ),
with the electrode pad and connector pins masked off with a small piece of
tape or a dab of removable maskant before coating.**

## Recommendation

**Silicone (MG Chemicals 422B) is the better fit for "damp fungal environment
+ wants to repair/service things later," with acrylic (MG 419D pen) as a
close, slightly-easier-to-rework second choice.** Reasoning, checked against
the product pages above rather than assumed:

- Both silicone and acrylic carry the same nominal fungus/moisture resistance
  claim, so that's not the deciding factor.
- Silicone's advantage here is thermal-cycling flexibility ("minimum stress
  on components during thermal cycling") and a softer, more compliant film --
  relevant because a flex PCB tab will flex in use and during handling for
  service, and a brittle acrylic film is more prone to cracking at flex
  points than an elastomeric silicone film. Cracked coating at a flex point
  is a moisture-ingress path right where you don't want one.
- Acrylic remains fully valid and is the *easier* of the two to strip for
  rework (plain solvents like acetone/MEK, or MG's dedicated 8309 remover,
  work faster on acrylic than on silicone) -- if repeated rework/rebuild
  cycles matter more to the user than long-term flex durability, pick
  acrylic instead. Both are pen-appliable and both leave the door open for
  future rework, unlike urethane (more solvent-resistant, harder to strip)
  or parylene (not hobbyist-reworkable at all, and not even hobbyist
  self-*applied* -- it requires sending the board to a CVD vendor).
- Urethane is not recommended here: its extra chemical resistance doesn't
  buy anything this application needs (nothing suggests the buffer board
  will see aggressive chemical exposure, just damp organic substrate air),
  and it actively works against the "keep it serviceable" goal.
- Parylene is not recommended despite having the best raw moisture/dielectric
  barrier of the four, specifically because it fails the project's stated
  constraint: it is not hobbyist-appliable (CVD vendor job) and not
  hobbyist-reworkable (insoluble at room temperature; industrial rework needs
  plasma etching, media blasting, or laser ablation). If a future revision
  of the buffer board is finalized, stable, and expected to never need
  in-field rework, parylene could be revisited for a "final" batch -- but
  that's explicitly not this project's current stage.

**Either way: mask the electrode sensing contact and the cable/header
connector pins before coating**, per the dielectric-properties section
above.

## Mycelium-specific consideration (honest gap, reasoned from general practice)

No datasheet or standard found addresses conformal coating specifically in
contact with or immediately adjacent to a living, actively growing mycelium
substrate -- the industry's "fungus resistance" testing (tied to
IPC-CC-830C, descending from the old MIL-I-46058C fungus-resistance test) is
about the cured coating film not supporting incidental mold/mildew growth
during humid storage or field service, not about long-term proximity to a
deliberately cultivated fungal organism. This is a genuine gap in available
literature; the following is reasoning from general conformal-coating
practice, not a cited mycology or materials study:

- **Coating doesn't attract or feed fungus, but it also doesn't repel it.**
  None of the general-purpose acrylic/silicone/urethane products reviewed
  here are marketed as biocidal or antimicrobial -- "fungus resistant" means
  the cured resin isn't a food source for mold spores, not that it actively
  inhibits fungal growth. SCS Coatings does sell a parylene variant
  (`microRESIST`) with an added biocide specifically for medical-device
  antimicrobial use, which underlines that antimicrobial behavior is a
  distinct, separately-marketed property -- ordinary conformal coatings don't
  have it. Expect mycelium to be indifferent to a coated surface it touches,
  neither harmed by it nor actively colonizing it faster than any other
  inert plastic surface it can grow across.
- **The likely real failure mode is edges and cut traces, not the flat
  field.** If hyphae or substrate moisture find their way under an unsealed
  board edge, connector back-shell, or a nick in the coating (e.g. from
  handling during "keep them alive" servicing), capillary wicking there is
  a more plausible long-term ingress path than the coating's bulk moisture
  permeability. Coat generously past the component field to the board edge,
  and re-coat (spot touch-up with the same pen) after any rework rather than
  leaving bare FR4/flex substrate exposed post-repair.
- **No evidence of the coating harming the fungal substrate at the contact
  interface either.** Cured acrylic/silicone/urethane films are chemically
  inert bulk polymers once fully cured (uncured solvent/VOC off-gassing
  during cure is the main chemical-exposure concern, and that's a short
  window, not a steady-state one); there's no known mechanism by which a
  fully-cured coating a few tens of microns thick would meaningfully change
  the substrate's chemistry at a distance. This is inference from general
  cured-polymer inertness, not a verified mycology finding -- if biological
  compatibility at the substrate interface turns out to matter for a future
  revision, that would need actual growth-trial observation on the specific
  species used (per `hardware/README.md`'s species table), not a coating
  datasheet.

Net: no direct literature exists for this specific pairing (conformal
coating + living mycelium substrate); treat the above as reasoned defaults
to validate empirically during the "build one pin board + one ADC module,
run for a few days on a live block" step already planned in
`hardware/README.md`, not as settled fact.
