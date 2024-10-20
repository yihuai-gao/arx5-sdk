from typing import Dict, List, Union
import numbers
from queue import Empty, Full
from multiprocessing.managers import SharedMemoryManager
import numpy as np
from shared_memory.shared_memory_util import ArraySpec, SharedAtomicCounter
from shared_memory.shared_ndarray import SharedNDArray


class SharedMemoryQueue:
    """
    A Lock-Free FIFO Shared Memory Data Structure.
    Stores a sequence of dict of numpy arrays.
    """

    class CallbackGuard:
        def __init__(self, callback, data):
            self.callback = callback
            self.data = data

        def __enter__(self):
            return self.data

        def __exit__(self, type, value, traceback):
            self.callback()
            del self.data
            del self.callback

    def __init__(
        self,
        shm_manager: SharedMemoryManager,
        array_specs: List[ArraySpec],
        buffer_size: int,
        use_atomic_counter: bool = True,
    ):
        self.use_atomic_counter = use_atomic_counter
        # create atomic counter
        if use_atomic_counter:
            write_counter = SharedAtomicCounter(shm_manager)
            read_counter = SharedAtomicCounter(shm_manager)
        else:
            write_counter = SharedCounter(shm_manager)
            read_counter = SharedCounter(shm_manager)

        # allocate shared memory
        shared_arrays = dict()
        for spec in array_specs:
            key = spec.name
            assert key not in shared_arrays
            array = SharedNDArray.create_from_shape(
                mem_mgr=shm_manager,
                shape=(buffer_size,) + tuple(spec.shape),
                dtype=spec.dtype,
            )
            shared_arrays[key] = array

        self.buffer_size = buffer_size
        self.array_specs = array_specs
        self.write_counter = write_counter
        self.read_counter = read_counter
        self.shared_arrays = shared_arrays

    @classmethod
    def create_from_examples(
        cls,
        shm_manager: SharedMemoryManager,
        examples: Dict[str, Union[np.ndarray, float]],
        buffer_size: int,
        use_atomic_counter: bool = True,
    ):
        specs = list()
        for key, value in examples.items():
            shape = None
            dtype = None
            if isinstance(value, np.ndarray):
                shape = value.shape
                dtype = value.dtype
                assert dtype != np.dtype("O")
            elif isinstance(value, numbers.Number):
                shape = tuple()
                dtype = np.dtype(type(value))
            else:
                raise TypeError(f"Unsupported type {type(value)}")

            spec = ArraySpec(name=key, shape=shape, dtype=dtype)
            specs.append(spec)

        obj = cls(
            shm_manager=shm_manager,
            array_specs=specs,
            buffer_size=buffer_size,
            use_atomic_counter=use_atomic_counter,
        )
        return obj

    def qsize(self):
        read_count = self.read_counter.load()
        write_count = self.write_counter.load()
        n_data = write_count - read_count
        return n_data

    def empty(self):
        n_data = self.qsize()
        return n_data <= 0

    def clear(self):
        self.read_counter.store(self.write_counter.load())

    def put(self, data: Dict[str, Union[np.ndarray, float]]):
        read_count = self.read_counter.load()
        write_count = self.write_counter.load()
        n_data = write_count - read_count
        if n_data >= self.buffer_size:
            raise Full()

        next_idx = write_count % self.buffer_size

        # write to shared memory
        for key, value in data.items():
            arr: np.ndarray
            arr = self.shared_arrays[key].get()
            if isinstance(value, np.ndarray):
                arr[next_idx] = value
            else:
                arr[next_idx] = np.array(value, dtype=arr.dtype)

        # update idx
        self.write_counter.add(1)

    def put_list(self, data: Dict[str, np.ndarray]):
        list_lens = []
        for key, value in data.items():
            assert isinstance(value, np.ndarray)
            list_lens.append(value.shape[0])
            assert value.shape[0] == list_lens[0], "Inconsistent list lengths"
        list_len = list_lens[0]

        read_count = self.read_counter.load()
        write_count = self.write_counter.load()
        n_data = write_count - read_count
        if n_data + list_len > self.buffer_size:
            raise Full()

        self._put_list_impl(write_count, data, list_len)
        # update idx
        self.write_counter.add(list_len)

    def get(self, out=None) -> Dict[str, np.ndarray]:
        write_count = self.write_counter.load()
        read_count = self.read_counter.load()
        n_data = write_count - read_count
        if n_data <= 0:
            raise Empty()

        if out is None:
            out = self._allocate_empty()

        next_idx = read_count % self.buffer_size
        for key, value in self.shared_arrays.items():
            arr = value.get()
            np.copyto(out[key], arr[next_idx])

        # update idx
        self.read_counter.add(1)
        return out

    def get_next_view(self) -> Dict[str, np.ndarray]:
        """
        Get reference to the next element to write
        for zero-copy writing
        """
        read_count = self.read_counter.load()
        write_count = self.write_counter.load()
        n_data = write_count - read_count
        if n_data >= self.buffer_size:
            raise Full()

        next_idx = write_count % self.buffer_size
        out = dict()
        for key, value in self.shared_arrays.items():
            arr = value.get()
            out[key] = arr[next_idx]

        return out

    def put_next_view(self, data: Dict[str, Union[np.ndarray, numbers.Number]]):
        """
        Used in conjunction with get_next_view
        for zero-copy writing
        """
        read_count = self.read_counter.load()
        write_count = self.write_counter.load()
        n_data = write_count - read_count
        if n_data >= self.buffer_size:
            raise Full()

        next_idx = write_count % self.buffer_size
        # write to shared memory
        for key, value in data.items():
            arr: np.ndarray
            arr = self.shared_arrays[key].get()
            if isinstance(value, np.ndarray):
                # assumed already written to the array
                pass
            else:
                arr[next_idx] = np.array(value, dtype=arr.dtype)

        # update idx
        self.write_counter.add(1)

    def get_view(self) -> CallbackGuard:
        write_count = self.write_counter.load()
        read_count = self.read_counter.load()
        n_data = write_count - read_count
        if n_data <= 0:
            raise Empty()

        next_idx = read_count % self.buffer_size
        data = dict()
        for key, value in self.shared_arrays.items():
            arr = value.get()
            data[key] = arr[next_idx]

        return self.CallbackGuard(callback=lambda: self.read_counter.add(1), data=data)

    def get_k(self, k, out=None) -> Dict[str, np.ndarray]:
        write_count = self.write_counter.load()
        read_count = self.read_counter.load()
        n_data = write_count - read_count
        if n_data <= 0:
            raise Empty()
        assert k <= n_data

        out = self._get_k_impl(k, read_count, out=out)
        self.read_counter.add(k)
        return out

    def get_all(self, out=None) -> Dict[str, np.ndarray]:
        write_count = self.write_counter.load()
        read_count = self.read_counter.load()
        n_data = write_count - read_count
        if n_data <= 0:
            raise Empty()

        out = self._get_k_impl(n_data, read_count, out=out)
        self.read_counter.add(n_data)
        return out

    def peek_all(self, out=None) -> Dict[str, np.ndarray]:
        """Peek all data without updating the read counter"""
        write_count = self.write_counter.load()
        read_count = self.read_counter.load()
        n_data = write_count - read_count
        if n_data <= 0:
            raise Empty()

        out = self._get_k_impl(n_data, read_count, out=out)
        return out

    def _get_k_impl(self, k, read_count, out=None) -> Dict[str, np.ndarray]:
        if out is None:
            out = self._allocate_empty(k)

        curr_idx = read_count % self.buffer_size
        for key, value in self.shared_arrays.items():
            arr = value.get()
            target = out[key]

            start = curr_idx
            end = min(start + k, self.buffer_size)
            target_start = 0
            target_end = end - start
            target[target_start:target_end] = arr[start:end]

            remainder = k - (end - start)
            if remainder > 0:
                # wrap around
                start = 0
                end = start + remainder
                target_start = target_end
                target_end = k
                target[target_start:target_end] = arr[start:end]

        return out

    def _put_list_impl(
        self, write_count: int, data: Dict[str, np.ndarray], list_len: int
    ):
        curr_idx = write_count % self.buffer_size
        # write to shared memory

        start = curr_idx
        end = min(start + list_len, self.buffer_size)
        data_start = 0
        data_end = end - start
        for key, value in data.items():
            arr: np.ndarray = self.shared_arrays[key].get()
            assert (
                arr.dtype == value.dtype
            ), f"Inconsistent data types: {arr.dtype} != {value.dtype}"
            arr[start:end] = value[data_start:data_end]

        remainder = list_len - (end - start)
        if remainder > 0:
            # wrap around
            start = 0
            end = start + remainder
            data_start = data_end
            data_end = list_len
            for key, value in data.items():
                arr: np.ndarray = self.shared_arrays[key].get()
                assert (
                    arr.dtype == value.dtype
                ), f"Inconsistent data types: {arr.dtype} != {value.dtype}"
                arr[start:end] = value[data_start:data_end]

    def _allocate_empty(self, k=None):
        result = dict()
        for spec in self.array_specs:
            shape = spec.shape
            if k is not None:
                shape = (k,) + shape
            result[spec.name] = np.empty(shape=shape, dtype=spec.dtype)
        return result
