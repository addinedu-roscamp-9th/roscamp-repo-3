from sqlalchemy.exc import OperationalError, SQLAlchemyError

from app.database import user_mapper
from app.database.connection import SessionLocal


def gui_connection_test():
    return "foobar"


async def query_user_details():
    # Prompt for user ID
    try:
        user_id = input("Enter user ID: ").strip()
    except (KeyboardInterrupt, EOFError):
        print("\n\nOperation cancelled by user.")
        return

    if not user_id:
        print("Error: User ID cannot be empty")
        return

    # Create database session
    db = SessionLocal()

    try:
        # Query user by ID
        user = user_mapper.select_user_by_id(user_id)

        if user:

            print("=" * 30)
            print("selected user")
            print(f"user_id: {user.user_id}")
            print(f"user_pw: {user.user_pw}")
            print(f"user_name: {user.user_name}")
            print("=" * 30)

        else:
            print(f"No user with: {user_id}")
    except OperationalError as e:
        print(f"\nDatabase connection error: {e}")
    except SQLAlchemyError as e:
        print(f"\nDatabase error: {e}")
    finally:
        db.close()
