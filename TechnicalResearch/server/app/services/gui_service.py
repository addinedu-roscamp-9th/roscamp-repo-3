from app.database import user_mapper


def login(data):
    user_id = data["id"]
    user_pw = data["pw"]

    user_name = user_mapper.login_user(user_id, user_pw)

    if user_name is not None:
        return user_name
    return False




