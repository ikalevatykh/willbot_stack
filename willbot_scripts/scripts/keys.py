import pickle as pkl
import numpy as np

class Keys:
    def __init__(self, keys=None):

        if isinstance(keys, list):
            self._keys = sorted(keys)
        else:
            raise TypeError('Input should be a list of keys.')

        self._demo2idx = None
        self._demos_max_step = None
        self._cumulative_max_step = None
        self._idx_ref = None
        self._category = None
        self._length = 0
        self._limit = ''
        self._split_keys()

    def __len__(self):
        return self._length

    def __getitem__(self, idx):
        return self._keys[idx]

    def __getstate__(self):
        return self._keys

    def __setstate__(self, keys):
        self.__init__(keys)

    def idx2demo(self, idx):
        assert idx < len(self)
        return self.key2demo(self._keys[idx])

    def key2demo(self, key):
        splits = key.split('/')
        demo, step = [int(split[1:]) for split in splits]
        return demo, step

    # split keys according to recorded trajectories
    # split for different #demo
    def _split_keys(self):
        keys = self._keys

        demo2idx = {}
        demos_max_step = {}
        for i, key in enumerate(keys):
            demo, step = self.key2demo(key)
            demo2idx[(demo, step)] = i
            if demo not in demos_max_step:
                demos_max_step[demo] = step+1
            else:
                demos_max_step[demo] = max(demos_max_step[demo], step+1)

        self._demo2idx = demo2idx
        self._demos_max_step = demos_max_step
        max_steps = [demos_max_step[key] for key in sorted(list(demos_max_step.keys()))]
        self._cumulative_max_step = np.cumsum(max_steps)
        self._length = len(demo2idx)

    def set_query_limit(self, limit):
        assert limit in ['', 'demo', 'skill']
        self._limit = limit

    def set_idx_reference(self, idx, category=None):
        assert idx < len(self),\
            'Index {} is outside of dataset with length {}'.format(idx, length)
        if category is not None:
            self._category = category
        self._idx_ref = idx

    def get_idx_min_max(self, idx_start, idx_end):
        assert idx_start < idx_end,\
            'idx_start {} idx_end {} length {}'.format(idx_start, idx_end, len(self))
        if self._limit:
            assert self._idx_ref is not None,\
                'A reference index needs to be specified for the query limit \'{}\''\
                    .format(self._limit)

        limit = self._limit
        idx_ref = self._idx_ref
        demos_max_step = self._demos_max_step
        category = self._category

        if limit == 'demo':
            demo, timestep = self.idx2demo(idx_ref)
            bound_min = idx_start-timestep
            bound_max = bound_min+demos_max_step[demo]
        elif limit == 'skill':
            demo, timestep = self.idx2demo(idx_ref)
            bound_min = idx_start-timestep+1
            bound_max = bound_min+demos_max_step[demo]-1
            idx_ref = max(bound_min, idx_ref)
            skills = np.array(category[idx_ref:bound_max])
            skill_ref = category[idx_ref]
            skills_class = skills == skill_ref
            max_idx_skill = skills_class.argmin()
            if max_idx_skill > 0:
                bound_max = idx_ref+max_idx_skill
        else:
            bound_min = 0
            bound_max = len(self)
        assert bound_min <= bound_max,\
            'bound min {} bound max {} limit {} demo {}, timestep {}'\
            .format(bound_min, bound_max, limit, demo, timestep)

        idx_min = max(idx_start, bound_min)
        idx_max = min(idx_end, bound_max)
        if idx_min == idx_max:
            idx_max += 1

        return idx_min, idx_max

    def set_max_demos(self, max_demos):
        cum_max_step = self._cumulative_max_step
        if max_demos is None:
            max_demos = len(cum_max_step)
        assert max_demos <= len(cum_max_step) and max_demos > 0
        self._length = cum_max_step[max_demos-1]

    def get_num_demos(self):
        return len(self._cumulative_max_step)

    def get_demo(self, demo_idx):
        cum_max_step = self._cumulative_max_step
        assert demo_idx < len(self)
        idx_beg = cum_max_step[demo_idx-1] if demo_idx > 0 else 0
        idx_end = cum_max_step[demo_idx]
        return slice(idx_beg, idx_end)
