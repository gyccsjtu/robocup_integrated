Generated worlds land here at run time (one layout per seed).

Nothing in this directory is an official competition map: every file is a
team training artefact produced by scripts/generate_training_city.py from
config/training_city.yaml. The official world, its randomisation script and
the judge interface have not been published yet.

Regenerate instead of editing:

    python3 ../scripts/generate_training_city.py --preset unit --seed 42 \
        --output ./training_city_unit_42.world
