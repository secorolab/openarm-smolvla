import sys
from lerobot.datasets.sampler import EpisodeAwareSampler


class FilteredEpisodeSampler(EpisodeAwareSampler):
    def __init__(
        self,
        dataset_from_indices,
        dataset_to_indices,
        episode_indices_to_use=None,
        drop_n_first_frames=0,
        drop_n_last_frames=0,
        shuffle=False,
        seed=0,
    ):
        starts = [int(x) for x in dataset_from_indices]
        ends = [int(x) for x in dataset_to_indices]

        if episode_indices_to_use is not None:
            selected = {int(x) for x in episode_indices_to_use}
            if any(i < 0 or i >= len(starts) for i in selected):
                raise ValueError("Invalid selected episode index")

            offset = 0
            for i in range(len(starts)):
                length = ends[i] - starts[i]
                if length < 0:
                    raise ValueError("Invalid episode boundaries")
                if i in selected:
                    starts[i] = offset
                    ends[i] = offset + length
                    offset += length

        super().__init__(
            starts,
            ends,
            episode_indices_to_use=episode_indices_to_use,
            drop_n_first_frames=drop_n_first_frames,
            drop_n_last_frames=drop_n_last_frames,
            shuffle=shuffle,
            seed=seed,
        )


def self_test():
    starts, ends = [0, 4, 8], [4, 8, 12]

    sampler = FilteredEpisodeSampler(starts, ends, [0, 2])
    assert list(sampler) == list(range(8))

    sampler = FilteredEpisodeSampler(
        starts, ends, [0, 2],
        drop_n_first_frames=1,
        drop_n_last_frames=1,
    )
    assert list(sampler) == [1, 2, 5, 6]

    sampler = FilteredEpisodeSampler(starts, ends)
    assert list(sampler) == list(range(12))

    sampler = FilteredEpisodeSampler(
        starts, ends, [0, 2], shuffle=True, seed=42
    )
    assert sorted(sampler) == list(range(8))

    print("PASS: filtered indices, episode boundaries and shuffle")


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        self_test()
    else:
        from lerobot.scripts import lerobot_train
        lerobot_train.EpisodeAwareSampler = FilteredEpisodeSampler
        lerobot_train.main()
