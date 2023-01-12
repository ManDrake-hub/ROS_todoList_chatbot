def log_exception(exception, exit=False):
    def decorator(func):
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except exception as e:
                print(f"Captured exception: {e}")
            finally:
                if exit:
                    raise e
    return decorator