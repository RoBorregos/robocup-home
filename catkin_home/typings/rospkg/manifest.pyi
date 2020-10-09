"""
This type stub file was generated by pyright.
"""

"""
Library for processing 'manifest' files, i.e. manifest.xml and
stack.xml.
"""
REQUIRED = ['license']
ALLOWXHTML = ['description']
OPTIONAL = ['author', 'logo', 'url', 'brief', 'description', 'status', 'notes', 'depend', 'rosdep', 'export', 'review', 'versioncontrol', 'platform', 'version', 'rosbuild2', 'catkin']
VALID = REQUIRED + OPTIONAL
class InvalidManifest(Exception):
    ...


class Export(object):
    """
    Manifest 'export' tag
    """
    def __init__(self, tag, attrs, str) -> None:
        """
        Create new export instance.
        :param tag: name of the XML tag
        @type  tag: str
        :param attrs: dictionary of XML attributes for this export tag
        @type  attrs: dict
        :param str: string value contained by tag, if any
        @type  str: str
        """
        ...
    
    def get(self, attr):
        """
        :returns: value of attribute or ``None`` if attribute not set, ``str``
        """
        ...
    


class Platform(object):
    """
    Manifest 'platform' tag
    """
    __slots__ = ...
    def __init__(self, os_, version, notes=...) -> None:
        """
        Create new depend instance.
        :param os_: OS name. must be non-empty, ``str``
        :param version: OS version. must be non-empty, ``str``
        :param notes: (optional) notes about platform support, ``str``
        """
        ...
    
    def __str__(self) -> str:
        ...
    
    def __repr__(self):
        ...
    
    def __eq__(self, obj) -> bool:
        """
        Override equality test. notes *are* considered in the equality test.
        """
        ...
    
    def __hash__(self) -> int:
        """
        :returns: an integer, which must be the same for two equal instances.

        Since __eq__ is defined, Python 3 requires that this class also provide a __hash__ method.
        """
        ...
    


class Depend(object):
    """
    Manifest 'depend' tag
    """
    __slots__ = ...
    def __init__(self, name, type_) -> None:
        """
        Create new depend instance.
        :param name: dependency name (e.g. package/stack). Must be non-empty
        @type  name: str
        :param type_: dependency type, e.g. 'package', 'stack'.  Must be non-empty.
        @type  type_: str

        @raise ValueError: if parameters are invalid
        """
        ...
    
    def __str__(self) -> str:
        ...
    
    def __repr__(self):
        ...
    
    def __eq__(self, obj) -> bool:
        ...
    
    def __hash__(self) -> int:
        """
        :returns: an integer, which must be the same for two equal instances.

        Since __eq__ is defined, Python 3 requires that this class also provide a __hash__ method.
        """
        ...
    


class RosDep(object):
    """
    Manifest 'rosdep' tag
    """
    __slots__ = ...
    def __init__(self, name) -> None:
        """
        Create new rosdep instance.

        :param name: dependency name. Must be non-empty. ``str``
        """
        ...
    


class Manifest(object):
    """
    Object representation of a ROS manifest file (``manifest.xml`` and ``stack.xml``)
    """
    __slots__ = ...
    def __init__(self, type_=..., filename=..., is_catkin=...) -> None:
        """
        :param type: `'package'` or `'stack'`
        :param filename: location of manifest file.  Necessary if
          converting ``${prefix}`` in ``<export>`` values, ``str``.
        """
        ...
    
    def get_export(self, tag, attr, convert=...):
        """
        :param tag: Name of XML tag to retrieve, ``str``
        :param attr: Name of XML attribute to retrieve from tag, ``str``
        :param convert: If ``True``, interpret variables (e.g. ``${prefix}``) export values.
        :returns: exports that match the specified tag and attribute, e.g. 'python', 'path'. ``[str]``
        """
        ...
    


_static_rosdep_view = None
def parse_manifest_file(dirpath, manifest_name, rospack=...):
    """
    Parse manifest file (package, stack).  Type will be inferred from manifest_name.

    :param dirpath: directory of manifest file, ``str``
    :param manifest_name: ``MANIFEST_FILE`` or ``STACK_FILE``, ``str``
    :param rospack: a RosPack instance to identify local packages as ROS packages

    :returns: return :class:`Manifest` instance, populated with parsed fields
    :raises: :exc:`InvalidManifest`
    :raises: :exc:`IOError`
    """
    ...

def parse_manifest(manifest_name, string, filename=...):
    """
    Parse manifest string contents.

    :param manifest_name: ``MANIFEST_FILE`` or ``STACK_FILE``, ``str``
    :param string: manifest.xml contents, ``str``
    :param filename: full file path for debugging, ``str``
    :returns: return parsed :class:`Manifest`
    """
    ...
