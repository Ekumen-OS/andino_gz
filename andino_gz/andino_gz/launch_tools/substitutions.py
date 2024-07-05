from typing import Dict, Iterable, Text, Union, List, Sequence
from launch.frontend import expose_substitution
from launch.substitution import Substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch.utilities.type_utils import perform_typed_substitution


class TextJoin(Substitution):
    """Substitution that TextJoin stuff."""

    def __init__(self,
                 substitutions: Iterable[Union[Text, Substitution]],
                 separator: Text = '') -> None:
        """Create a TextJoin."""
        self.__substitutions = normalize_to_list_of_substitutions(
            substitutions)
        self.__separator = separator

    @property
    def substitutions(self) -> Iterable[Substitution]:
        """Getter for variable_name."""
        return self.__substitutions

    @property
    def separator(self) -> Text:
        """Getter for variable_name."""
        return self.__separator

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"TextJoin: {self.__separator.join([sub.describe() for sub in self.__substitutions])}"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by retrieving the local variable."""
        performed_substitutions = [
            sub.perform(context) for sub in self.__substitutions
        ]
        return self.__separator.join(performed_substitutions)
