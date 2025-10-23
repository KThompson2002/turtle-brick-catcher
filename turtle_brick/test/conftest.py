"""Ensure all tests run and configure multiprocessing module."""
import multiprocessing
multiprocessing.set_start_method('spawn', force=True)
