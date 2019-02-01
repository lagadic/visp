#!/usr/bin/env python

import sys, re, os.path, errno, fnmatch
import json
import logging
from shutil import copyfile
from pprint import pformat
from string import Template

if sys.version_info[0] >= 3:
    from io import StringIO
else:
    from cStringIO import StringIO

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# list of modules + files remap
config = None
ROOT_DIR = None
FILES_REMAP = {}
def checkFileRemap(path):
    path = os.path.realpath(path)
    if path in FILES_REMAP:
        return FILES_REMAP[path]
    assert path[-3:] != '.in', path
    return path


total_files = 0
updated_files = 0

module_imports = []
module_j_code = None
module_jn_code = None

# list of class names, which should be skipped by wrapper generator
# the list is loaded from misc/java/gen_dict.json defined for the module and its dependencies
class_ignore_list = []

# list of constant names, which should be skipped by wrapper generator
# ignored constants can be defined using regular expressions
const_ignore_list = []

# list of private constants
const_private_list = []

# { Module : { public : [[name, val],...], private : [[]...] } }
missing_consts = {}

# c_type    : { java/jni correspondence }
# Complex data types are configured for each module using misc/java/gen_dict.json

type_dict = {
    # "simple"  : { j_type : "?", jn_type : "?", jni_type : "?", suffix : "?" },
    "": {"j_type": "", "jn_type": "long", "jni_type": "jlong"},  # c-tor ret_type
    "void": {"j_type": "void", "jn_type": "void", "jni_type": "void"},
    "env": {"j_type": "", "jn_type": "", "jni_type": "JNIEnv*"},
    "cls": {"j_type": "", "jn_type": "", "jni_type": "jclass"},
    "bool": {"j_type": "boolean", "jn_type": "boolean", "jni_type": "jboolean", "suffix": "Z"},
    "char": {"j_type": "char", "jn_type": "char", "jni_type": "jchar", "suffix": "C"},
    "int": {"j_type": "int", "jn_type": "int", "jni_type": "jint", "suffix": "I"},
    "long": {"j_type": "int", "jn_type": "int", "jni_type": "jint", "suffix": "I"},
    "float": {"j_type": "float", "jn_type": "float", "jni_type": "jfloat", "suffix": "F"},
    "double": {"j_type": "double", "jn_type": "double", "jni_type": "jdouble", "suffix": "D"},
    "size_t": {"j_type": "long", "jn_type": "long", "jni_type": "jlong", "suffix": "J"},
    "__int64": {"j_type": "long", "jn_type": "long", "jni_type": "jlong", "suffix": "J"},
    "int64": {"j_type": "long", "jn_type": "long", "jni_type": "jlong", "suffix": "J"},
    "double[]": {"j_type": "double[]", "jn_type": "double[]", "jni_type": "jdoubleArray", "suffix": "_3D"}
}

# { class : { func : {j_code, jn_code, cpp_code} } }
ManualFuncs = {}
ToStringSupport = []

# { class : { func : { arg_name : {"ctype" : ctype, "attrib" : [attrib]} } } }
func_arg_fix = {}

'''
    # INFO: Needs no comments :D
'''


def read_contents(fname):
    with open(fname, 'r') as f:
        data = f.read()
    return data


'''
    # INFO: Create dir or a file if not created already
'''


def mkdir_p(path):
    ''' mkdir -p '''
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def camelCase(s):
    '''
        turns vpHomoMatrix to VpHomoMatrix
    '''
    if len(s) > 0:
        return s[0].upper() + s[1:]
    else:
        return s


def reverseCamelCase(s):
    '''
        turns VpHomoMatrix to vpHomoMatrix
    '''
    if len(s) > 0:
        return s[0].lower() + s[1:]
    else:
        return s


T_JAVA_START_INHERITED = read_contents(os.path.join(SCRIPT_DIR, 'templates/java_class_inherited.prolog'))
T_JAVA_START_ORPHAN = read_contents(os.path.join(SCRIPT_DIR, 'templates/java_class.prolog'))
T_JAVA_START_MODULE = read_contents(os.path.join(SCRIPT_DIR, 'templates/java_module.prolog'))
T_CPP_MODULE = Template(read_contents(os.path.join(SCRIPT_DIR, 'templates/cpp_module.template')))


class GeneralInfo():
    def __init__(self, type, decl, namespaces):
        self.namespace, self.classpath, self.classname, self.name = self.parseName(decl[0], namespaces)

        # parse doxygen comments
        self.params = {}
        self.annotation = []
        if type == "class":
            docstring = "// C++: class " + self.name + "\n//javadoc: " + self.name
        else:
            docstring = ""
        if len(decl) > 5 and decl[5]:
            # logging.info('docstring: %s', decl[5])
            if re.search("(@|\\\\)deprecated", decl[5]):
                self.annotation.append("@Deprecated")

        self.docstring = docstring

    def parseName(self, name, namespaces):
        '''
        input: full name and available namespaces
        returns: (namespace, classpath, classname, name)
        '''
        name = name[name.find(" ") + 1:].strip()  # remove struct/class/const prefix
        spaceName = ""
        localName = name  # <classes>.<name>

        if name.count('.') == 1 and len(namespaces) == 1:
            namespaces = list(namespaces)
            pieces = name.split(".")
            return namespaces[0], ".".join(namespaces[:-1]), camelCase(pieces[0]), pieces[1]

        for namespace in sorted(namespaces, key=len, reverse=True):
            if name.startswith(namespace + "."):
                spaceName = namespace
                localName = name.replace(namespace + ".", "")
                break
        pieces = localName.split(".")
        if len(pieces) > 2:  # <class>.<class>.<class>.<name>
            return spaceName, ".".join(pieces[:-1]), pieces[-2], pieces[-1]
        elif len(pieces) == 2:  # <class>.<name>
            return spaceName, pieces[0], pieces[0], pieces[1]
        elif len(pieces) == 1:  # <name>
            return spaceName, "", "", pieces[0]
        else:
            return spaceName, "", ""  # error?!

    def fullName(self, isCPP=False):
        result = ".".join([self.fullClass(), reverseCamelCase(self.name)])
        return result if not isCPP else result.replace(".", "::")

    def fullClass(self, isCPP=False):
        result = ".".join([f for f in [self.namespace] + self.classpath.split(".") if len(f) > 0])
        return result if not isCPP else result.replace(".", "::")


class ConstInfo(GeneralInfo):
    def __init__(self, decl, addedManually=False, namespaces=[]):
        GeneralInfo.__init__(self, "const", decl, namespaces)
        self.cname = self.name.replace(".", "::")
        self.value = decl[1]
        self.addedManually = addedManually

    def __repr__(self):
        return Template("CONST $name=$value$manual").substitute(name=self.name,
                                                                value=self.value,
                                                                manual="(manual)" if self.addedManually else "")

    def isIgnored(self):
        for c in const_ignore_list:
            if re.match(c, self.name):
                return True
        return False


class ClassPropInfo():
    def __init__(self, decl):  # [f_ctype, f_name, '', '/RW']
        self.ctype = decl[0]
        self.name = decl[1]
        self.rw = "/RW" in decl[3]

    def __repr__(self):
        return Template("PROP $ctype $name").substitute(ctype=self.ctype, name=self.name)


class ClassInfo(GeneralInfo):
    def __init__(self, decl, namespaces=[]):  # [ 'class/struct cname', ': base', [modlist] ]
        GeneralInfo.__init__(self, "class", decl, namespaces)
        self.cname = self.name.replace(".", "::")
        self.methods = []
        self.methods_suffixes = {}
        self.consts = []  # using a list to save the occurrence order
        self.private_consts = []  # TODO: ViSP wont need these
        self.imports = set()
        self.props = []
        self.jname = self.name
        self.smart = None  # True if class stores Ptr<T>* instead of T* in nativeObj field
        self.j_code = None  # java code stream
        self.jn_code = None  # jni code stream
        self.cpp_code = None  # cpp code stream
        for m in decl[2]:
            if m.startswith("="):
                self.jname = m[1:]
        self.base = ''
        if decl[1]:
            # self.base = re.sub(r"\b"+self.jname+r"\b", "", decl[1].replace(":", "")).strip()
            self.base = re.sub(r"^.*:", "", decl[1].split(",")[0]).strip().replace(self.jname, "")

    def __repr__(self):
        return Template("CLASS $namespace::$classpath.$name : $base").substitute(**self.__dict__)

    def getAllImports(self, module):
        return ["import %s;" % c for c in sorted(self.imports) if not c.startswith('org.visp.' + module)]

    def addImports(self, ctype):
        if ctype in type_dict:
            if "j_import" in type_dict[ctype]:
                self.imports.add(type_dict[ctype]["j_import"])
            if "v_type" in type_dict[ctype]:
                self.imports.add("java.util.List")
                self.imports.add("java.util.ArrayList")
                self.imports.add("org.visp.utils.Converters")
                if type_dict[ctype]["v_type"] in ("VpMatrix", "vector_Mat"):
                    self.imports.add("org.visp.core.VpMatrix")

    def getAllMethods(self):
        result = []
        result.extend([fi for fi in sorted(self.methods) if fi.isconstructor])
        result.extend([fi for fi in sorted(self.methods) if not fi.isconstructor])
        return result

    def addMethod(self, fi):
        if fi not in self.methods: # Dont add duplicates
            self.methods.append(fi)

    def getConst(self, name):
        for cand in self.consts + self.private_consts:
            if cand.name == name:
                return cand
        return None

    def addConst(self, constinfo):
        # choose right list (public or private)
        consts = self.consts
        for c in const_private_list:
            if re.match(c, constinfo.name):
                consts = self.private_consts
                break
        consts.append(constinfo)

    def initCodeStreams(self, Module):
        self.j_code = StringIO()
        self.jn_code = StringIO()
        self.cpp_code = StringIO();

        # INFO: All of these T_JAVA are templates for starting portion of Java classes
        if self.base:
            self.j_code.write(T_JAVA_START_INHERITED)
        else:
            if self.name != Module:
                self.j_code.write(T_JAVA_START_ORPHAN)
            else:
                self.j_code.write(T_JAVA_START_MODULE)

        # INFO: Misc folder handling for modules like Core, Imgproc
        if self.name == Module:

            # INFO: module_imports - read from gen_dict.json. Contains 'java.lang.String', 'java.lang.Character' etc
            for i in module_imports or []:
                self.imports.add(i)
            if module_j_code:
                self.j_code.write(module_j_code)
            if module_jn_code:
                self.jn_code.write(module_jn_code)

    def cleanupCodeStreams(self):
        self.j_code.close()
        self.jn_code.close()
        self.cpp_code.close()

    def generateJavaCode(self, m, M):
        return Template(self.j_code.getvalue() + "\n\n" + \
                        self.jn_code.getvalue() + "\n}\n").substitute( \
            module=m,
            name=self.name,
            jname=self.jname,
            imports="\n".join(self.getAllImports(M)),
            docs=self.docstring,
            annotation="\n".join(self.annotation),
            base=self.base)

    def generateCppCode(self):
        return self.cpp_code.getvalue()


class ArgInfo():
    def __init__(self, arg_tuple):  # [ ctype, name, def val, [mod], argno ]
        self.pointer = False
        ctype = arg_tuple[0]
        if ctype.endswith("*"):
            ctype = ctype[:-1]
            self.pointer = True
        self.ctype = ctype
        self.name = arg_tuple[1]
        self.defval = arg_tuple[2]
        self.out = ""
        if "/O" in arg_tuple[3]:
            self.out = "O"
        if "/IO" in arg_tuple[3]:
            self.out = "IO"

    def __repr__(self):
        return Template("ARG $ctype$p $name=$defval").substitute(ctype=self.ctype,
                                                                 p=" *" if self.pointer else "",
                                                                 name=self.name,
                                                                 defval=self.defval)
    def __eq__(self, other):
        return self.ctype == other.ctype


class FuncInfo(GeneralInfo):
    def __init__(self, decl, namespaces=[]):  # [ funcname, return_ctype, [modifiers], [args] ]
        GeneralInfo.__init__(self, "func", decl, namespaces)
        self.cname = self.name.replace(".", "::")
        self.jname = self.name
        self.isconstructor = self.name == self.classname

        # TODO open-cv didn't add camel case support
        # I'm adding
        self.classname = camelCase(self.classname)
        self.classpath = camelCase(self.classpath)
        if self.isconstructor:
            self.name = camelCase(self.name)
            self.jname = camelCase(self.jname)

        if "[" in self.name:
            self.jname = "getelem"
        for m in decl[2]:
            if m.startswith("="):
                self.jname = m[1:]
        self.static = ["", "static"]["/S" in decl[2]]
        self.ctype = re.sub(r"^VpTermCriteria", "TermCriteria", decl[1] or "")
        self.args = []
        func_fix_map = func_arg_fix.get(self.jname, {})
        for a in decl[3]:
            arg = a[:]
            arg_fix_map = func_fix_map.get(arg[1], {})
            arg[0] = arg_fix_map.get('ctype', arg[0])  # fixing arg type
            arg[3] = arg_fix_map.get('attrib', arg[3])  # fixing arg attrib
            self.args.append(ArgInfo(arg))

    def __repr__(self):
        return Template("FUNC <$ctype $namespace.$classpath.$name $args>").substitute(**self.__dict__)

    def __lt__(self, other):
        return self.__repr__() < other.__repr__()

    def __eq__(self, other):
        return self.cname == other.cname and len(self.args) == len(other.args) and self.args == other.args


class JavaWrapperGenerator(object):
    def __init__(self):
        self.cpp_files = []
        self.clear()

    def clear(self):
        self.namespaces = set(["vp"])
        self.classes = {}
        self.module = ""
        self.Module = ""
        self.ported_func_list = []
        self.skipped_func_list = []
        self.def_args_hist = {}  # { def_args_cnt : funcs_cnt }

    def add_class(self, decl):
        classinfo = ClassInfo(decl, namespaces=self.namespaces)
        if classinfo.name in class_ignore_list:
            logging.info('ignored: %s', classinfo)
            return
        name = classinfo.name
        if self.isWrapped(name) and not classinfo.base:
            logging.warning('duplicated: %s', classinfo)
            return
        self.classes[name] = classinfo
        if name in type_dict and not classinfo.base:
            logging.warning('duplicated: %s', classinfo)
            return
        type_dict.setdefault(name, {}).update(
            {"j_type": classinfo.jname,
             "jn_type": "long", "jn_args": (("__int64", ".nativeObj"),),
             "jni_name": "(*(" + classinfo.fullName(isCPP=True) + "*)%(n)s_nativeObj)", "jni_type": "jlong",
             "suffix": "J",
             "j_import": "org.visp.%s.%s" % (self.module, classinfo.jname)
             }
        )
        type_dict.setdefault(name + '*', {}).update(
            {"j_type": classinfo.jname,
             "jn_type": "long", "jn_args": (("__int64", ".nativeObj"),),
             "jni_name": "(" + classinfo.fullName(isCPP=True) + "*)%(n)s_nativeObj", "jni_type": "jlong",
             "suffix": "J",
             "j_import": "org.visp.%s.%s" % (self.module, classinfo.jname)
             }
        )

        # missing_consts { Module : { public : [[name, val],...], private : [[]...] } }
        if name in missing_consts:
            if 'private' in missing_consts[name]:
                for (n, val) in missing_consts[name]['private']:
                    classinfo.private_consts.append(ConstInfo([n, val], addedManually=True))
            if 'public' in missing_consts[name]:
                for (n, val) in missing_consts[name]['public']:
                    classinfo.consts.append(ConstInfo([n, val], addedManually=True))

        # class props
        for p in decl[3]:
            if True:  # "vector" not in p[0]:
                classinfo.props.append(ClassPropInfo(p))
            else:
                logging.warning("Skipped property: [%s]" % name, p)

        if classinfo.base:
            classinfo.addImports(classinfo.base)
        type_dict.setdefault("Ptr_" + name, {}).update(
            {"j_type": classinfo.jname,
             "jn_type": "long", "jn_args": (("__int64", ".getNativeObjAddr()"),),
             "jni_name": "*((Ptr<" + classinfo.fullName(isCPP=True) + ">*)%(n)s_nativeObj)", "jni_type": "jlong",
             "suffix": "J",
             "j_import": "org.visp.%s.%s" % (self.module, classinfo.jname)
             }
        )
        logging.info('ok: class %s, name: %s, base: %s', classinfo, name, classinfo.base)

    def add_const(self, decl):  # [ "const cname", val, [], [] ]
        constinfo = ConstInfo(decl, namespaces=self.namespaces)
        if constinfo.isIgnored():
            logging.info('ignored: %s', constinfo)
        elif not self.isWrapped(constinfo.classname):
            logging.info('class not found: %s', constinfo)
        else:
            ci = self.getClass(constinfo.classname)
            duplicate = ci.getConst(constinfo.name)
            if duplicate:
                if duplicate.addedManually:
                    logging.info('manual: %s', constinfo)
                else:
                    logging.warning('duplicated: %s', constinfo)
            else:
                ci.addConst(constinfo)
                logging.info('ok: %s', constinfo)

    # INFO: Some functions are to be ignored - either they can't exist in java or have to be implemtd manully
    def add_func(self, decl):
        fi = FuncInfo(decl, namespaces=self.namespaces)
        classname = fi.classname or self.Module

        # Workaround for imgrpoc module
        if classname == 'Vp' and self.Module == 'Imgproc':
            classname = 'VpImgproc'

        if classname in class_ignore_list:
            logging.info('ignored: %s', fi)
        elif classname in ManualFuncs and fi.jname in ManualFuncs[classname]:
            logging.info('manual: %s', fi)
        elif not self.isWrapped(classname):
            logging.warning('not found: %s', fi)
        else:
            self.getClass(classname).addMethod(fi)
            logging.info('ok: %s', fi)
            # calc args with def val
            cnt = len([a for a in fi.args if a.defval])
            self.def_args_hist[cnt] = self.def_args_hist.get(cnt, 0) + 1

    def save(self, path, buf):
        global total_files, updated_files
        total_files += 1
        if os.path.exists(path):
            with open(path, "rt") as f:
                content = f.read()
                if content == buf:
                    return
        with open(path, "wt") as f:
            f.write(buf)
        updated_files += 1

    def gen(self, srcfiles, module, output_path, output_jni_path, output_java_path, common_headers):
        self.clear()
        self.module = module
        self.Module = module.capitalize()

        # INFO: In open-cv 4, hdr_parser is imported to gen2.py which is imported to gen_java
        parser = hdr_parser.CppHeaderParser()

        # INFO: Donno why open-cv was adding module in list of classes
        # self.add_class( ['class ' + self.Module, '', [], []] ) # [ 'class/struct cname', ':bases', [modlist] [props] ]

        # scan the headers and build more descriptive maps of classes, consts, functions
        includes = [];

        # INFO: Don't know yet what it does
        for hdr in common_headers:
            logging.info("\n===== Common header : %s =====", hdr)
            includes.append('#include "' + hdr + '"')

        # INFO: These are .hpp files containing enums, template classes and some VP_EXPORT functions
        for hdr in srcfiles:

            '''
                # INFO: decls contains enum and const declared in .hpp files. It also contains function declarations,
                but only those which have a VISP_EXPORT macro with them. Read a snippet from `hdr_parser.py` for more

                Each declaration is [funcname, return_value_type /* in C, not in Python */, <list_of_modifiers>, <list_of_arguments>, original_return_type, docstring],
                where each element of <list_of_arguments> is 4-element list itself:
                [argtype, argname, default_value /* or "" if none */, <list_of_modifiers>]
                where the list of modifiers is yet another nested list of strings
                   (currently recognized are "/O" for output argument, "/S" for static (i.e. class) methods
                   and "/A value" for the plain C arrays with counters)
                original_return_type is None if the original_return_type is the same as return_value_type

                Ex:
                ['const cv.Error.StsOk', '0', [], [], None, '']   , where cv and Error are namespaces
                ['cv.cubeRoot', 'float', [], [['float', 'val', '', []]], 'float', '@brief Computes the cube root of an ...']

            '''
            decls = parser.parse(hdr)

            # TODO: Above parse function detects functions but not classes.
            # SO I'm adding classes assuming that the file name is a class name
            # Find the last occurence of / in path/to/class/myClass.h
            # -2 for removing `.h`
            self.add_class(["class " + camelCase(hdr[hdr.rfind('/') + 1:-2]), '', [], []])

            # INFO: List of all namespaces mentioned in the .hpp file. Like cv,ogl,cuda etc
            self.namespaces = parser.namespaces
            logging.info("\n\n===== Header: %s =====", hdr)
            logging.info("Namespaces: %s", parser.namespaces)

            if decls:
                includes.append('#include "' + hdr + '"')
            else:
                logging.info("Ignore header: %s", hdr)

            # INFO: Now split each declaration in categories - class, const or function
            # INFO: Some functions .are to be ignored - either they can't exist in java or have to be implemtd manully
            for decl in decls:
                logging.info("\n--- Incoming ---\n%s", pformat(decl[:5], 4))  # without docstring
                name = decl[0]

                # ToDO ignore operator member functions
                if decl[1] == 'operator':
                    print("Skipped operator function: %s" % name)
                elif name.startswith("struct") or name.startswith("class"):
                    self.add_class(decl)
                elif name.startswith("const"):
                    self.add_const(decl)
                else:  # function
                    self.add_func(decl)

                logging.info("\n\n===== Generating... =====")
        moduleCppCode = StringIO()
        package_path = os.path.join(output_java_path, module)
        mkdir_p(package_path)
        for ci in self.classes.values():
            # INFO: Ignore classes that are manually. For open-cv it was Mat, for Visp it'll be vpMat and vpImage
            if ci.name in ["vpMatrix", "vpImageUChar", "vpImageRGBa", "vpArray2D"]:
                continue
            ci.initCodeStreams(self.Module)
            self.gen_class(ci)
            classJavaCode = ci.generateJavaCode(self.module, self.Module)
            self.save("%s/%s/%s.java" % (output_java_path, module, ci.jname), classJavaCode)
            moduleCppCode.write(ci.generateCppCode())
            ci.cleanupCodeStreams()
        cpp_file = os.path.abspath(os.path.join(output_jni_path, module + ".inl.hpp"))
        self.cpp_files.append(cpp_file)
        self.save(cpp_file, T_CPP_MODULE.substitute(m=module, M=module.upper(), code=moduleCppCode.getvalue(),
                                                    includes="\n".join(includes)))
        self.save(os.path.join(output_path, module + ".txt"), self.makeReport())

    def makeReport(self):
        '''
        Returns string with generator report
        '''
        report = StringIO()
        total_count = len(self.ported_func_list) + len(self.skipped_func_list)
        report.write("PORTED FUNCs LIST (%i of %i):\n\n" % (len(self.ported_func_list), total_count))
        report.write("\n".join(self.ported_func_list))
        report.write("\n\nSKIPPED FUNCs LIST (%i of %i):\n\n" % (len(self.skipped_func_list), total_count))
        report.write("".join(self.skipped_func_list))
        for i in self.def_args_hist.keys():
            report.write("\n%i def args - %i funcs" % (i, self.def_args_hist[i]))
        return report.getvalue()

    def fullTypeName(self, t):
        if self.isWrapped(t):
            return self.getClass(t).fullName(isCPP=True)
        else:
            return t

    def gen_func(self, ci, fi, prop_name=''):
        logging.info("%s", fi)
        j_code = ci.j_code
        jn_code = ci.jn_code
        cpp_code = ci.cpp_code

        # c_decl
        # e.g: void add(Mat src1, Mat src2, Mat dst, Mat mask = Mat(), int dtype = -1)
        if prop_name:
            c_decl = "%s %s::%s" % (fi.ctype, fi.classname, prop_name)
        else:
            decl_args = []
            for a in fi.args:  # INFO: Even arguments have their own class: [type, defval, name, isPointer(bool), out(ret type)]
                s = a.ctype or ' _hidden_ '
                if a.pointer:
                    s += "*"
                elif a.out:
                    s += "&"
                s += " " + a.name
                if a.defval:
                    s += " = " + a.defval
                decl_args.append(s)
            c_decl = "%s %s %s(%s)" % (fi.static, fi.ctype, fi.cname, ", ".join(decl_args))

        # INFO: This is the java comment above every function
        j_code.write("\n    //\n    // C++: %s\n    //\n\n" % c_decl)

        # check if we 'know' all the types
        if fi.ctype not in type_dict:  # unsupported ret type
            msg = "// Return type '%s' is not supported, skipping the function\n\n" % fi.ctype
            self.skipped_func_list.append(c_decl + "\n" + msg)
            j_code.write(" " * 4 + msg)
            logging.warning("SKIP:" + c_decl.strip() + "\t due to RET type" + fi.ctype)
            return

        for a in fi.args:
            if a.ctype not in type_dict:
                if not a.defval and a.ctype.endswith("*"):
                    a.defval = 0
                if a.defval:
                    a.ctype = ''
                    continue
                msg = "// Unknown type '%s' (%s), skipping the function\n\n" % (a.ctype, a.out or "I")
                self.skipped_func_list.append(c_decl + "\n" + msg)
                j_code.write(" " * 4 + msg)
                logging.warning("SKIP:" + c_decl.strip() + "\t due to ARG type" + a.ctype + "/" + (a.out or "I"))
                return

        self.ported_func_list.append(c_decl)

        # jn & cpp comment
        jn_code.write("\n    // C++: %s\n" % c_decl)
        cpp_code.write("\n//\n// %s\n//\n" % c_decl)

        # INFO: Generating the JNI method signatures
        args = fi.args[:]  # copy
        j_signatures = []
        suffix_counter = int(ci.methods_suffixes.get(fi.jname, -1))
        while True:
            suffix_counter += 1
            ci.methods_suffixes[fi.jname] = suffix_counter
            # java native method args
            jn_args = []
            # jni (cpp) function args
            jni_args = [ArgInfo(["env", "env", "", [], ""]), ArgInfo(["cls", "", "", [], ""])]
            j_prologue = []
            j_epilogue = []
            c_prologue = []
            c_epilogue = []

            # Add 3rd party specific tags
            # If Gsl or Lapack or OpenCV is missing, don't include them to prevent compilation error
            if fi.name in ['detByLUGsl',    'svdGsl',    'inverseByLUGsl',    'pseudoInverseGsl',    'inverseByGsl']:
                c_prologue.append('#if defined(VISP_HAVE_GSL)')

            if fi.name in ['detByLUOpenCV', 'svdOpenCV', 'inverseByLUOpenCV', 'pseudoInverseOpenCV', 'inverseByOpenCV', 'inverseByCholeskyOpenCV']:
                c_prologue.append('#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)')

            if fi.name in ['detByLUEigen3', 'svdEigen3', 'inverseByLUEigen3', 'pseudoInverseEigen3', 'inverseByEigen3']:
                c_prologue.append('#if defined(VISP_HAVE_EIGEN3)')

            if fi.name in ['detByLULapack', 'svdLapack', 'inverseByLULapack', 'pseudoInverseLapack', 'inverseByLapack', 'inverseByCholeskyLapack', 'inverseByQRLapack']:
                c_prologue.append('#if defined(VISP_HAVE_LAPACK)')

            if type_dict[fi.ctype]["jni_type"] == "jdoubleArray" and type_dict[fi.ctype]["suffix"] != "[D":
                fields = type_dict[fi.ctype]["jn_args"]
                c_epilogue.append( \
                    ("jdoubleArray _da_retval_ = env->NewDoubleArray(%(cnt)i);  " +
                     "jdouble _tmp_retval_[%(cnt)i] = {%(args)s}; " +
                     "env->SetDoubleArrayRegion(_da_retval_, 0, %(cnt)i, _tmp_retval_);") %
                    {"cnt": len(fields), "args": ", ".join(["(jdouble)_retval_" + f[1] for f in fields])})
            if fi.classname and fi.ctype and not fi.static:  # non-static class method except c-tor
                # adding 'self'
                jn_args.append(ArgInfo(["__int64", "nativeObj", "", [], ""]))
                jni_args.append(ArgInfo(["__int64", "self", "", [], ""]))
            ci.addImports(fi.ctype)
            for a in args:
                if not a.ctype:  # hidden
                    continue
                ci.addImports(a.ctype)
                if "v_type" in type_dict[a.ctype]:  # pass as vector
                    if type_dict[a.ctype]["v_type"] in ("Mat", "vector_Mat"):  # pass as Mat or vector_Mat
                        jn_args.append(ArgInfo(["__int64", "%s_mat.nativeObj" % a.name, "", [], ""]))
                        jni_args.append(ArgInfo(["__int64", "%s_mat_nativeObj" % a.name, "", [], ""]))
                        c_prologue.append(type_dict[a.ctype]["jni_var"] % {"n": a.name} + ";")
                        c_prologue.append("Mat& %(n)s_mat = *((Mat*)%(n)s_mat_nativeObj)" % {"n": a.name} + ";")
                        if "I" in a.out or not a.out:
                            if type_dict[a.ctype]["v_type"] == "vector_Mat":
                                j_prologue.append(
                                    "List<VpMatrix> %(n)s_tmplm = new ArrayList<VpMatrix>((%(n)s != null) ? %(n)s.size() : 0);" % {
                                        "n": a.name})
                                j_prologue.append(
                                    "Mat %(n)s_mat = Converters.%(t)s_to_Mat(%(n)s, %(n)s_tmplm);" % {"n": a.name,
                                                                                                      "t": a.ctype})
                            else:
                                j_prologue.append("Mat %(n)s_mat = Converters.%(t)s_to_Mat(%(n)s);" % {"n": a.name, "t": a.ctype})
                            c_prologue.append("Mat_to_%(t)s( %(n)s_mat, %(n)s );" % {"n": a.name, "t": a.ctype})
                        else:
                            j_prologue.append("Mat %s_mat = new Mat();" % a.name)
                        if "O" in a.out:
                            j_epilogue.append("Converters.Mat_to_%(t)s(%(n)s_mat, %(n)s);" % {"t": a.ctype, "n": a.name})
                            j_epilogue.append("%s_mat.release();" % a.name)
                            c_epilogue.append("%(t)s_to_Mat( %(n)s, %(n)s_mat );" % {"n": a.name, "t": a.ctype})
                    elif type_dict[a.ctype]["v_type"] in ("std::vector<double>"):
                        c_prologue.append("std::vector<double> v_ = List_to_vector_double(env, v);")
                    else:  # pass as list
                        jn_args.append(ArgInfo([a.ctype, a.name, "", [], ""]))
                        jni_args.append(ArgInfo([a.ctype, "%s_list" % a.name, "", [], ""]))
                        ci.addImports(a.ctype)
                        j_prologue.append("long[] "+a.name+" = Converters."+a.ctype+"_to_Array("+a.name+"_list_arr);")
                        a.name += '_list'
                        c_prologue.append(type_dict[a.ctype]["jni_var"] % {"n": a.name} + "_arr;")
                        if "I" in a.out or not a.out:
                            c_prologue.append("%(n)s_arr = List_to_%(t)s(env, %(n)s);" % {"n": a.name, "t": a.ctype})
                        if "O" in a.out:
                            c_epilogue.append("Copy_%s_to_List(env,%s,%s_list);" % (a.ctype, a.name, a.name))
                        a.name += '_arr'
                else:
                    fields = type_dict[a.ctype].get("jn_args", ((a.ctype, ""),))
                    if "I" in a.out or not a.out or self.isWrapped(a.ctype):  # input arg, pass by primitive fields
                        for f in fields:
                            jn_args.append(ArgInfo([f[0], a.name + f[1], "", [], ""]))
                            jni_args.append(ArgInfo([f[0], a.name + f[1].replace(".", "_").replace("[", "").replace("]",
                                                                                                                    "").replace(
                                "_getNativeObjAddr()", "_nativeObj"), "", [], ""]))
                    if "O" in a.out and not self.isWrapped(a.ctype):  # out arg, pass as double[]
                        jn_args.append(ArgInfo(["double[]", "%s_out" % a.name, "", [], ""]))
                        jni_args.append(ArgInfo(["double[]", "%s_out" % a.name, "", [], ""]))
                        j_prologue.append("double[] %s_out = new double[%i];" % (a.name, len(fields)))
                        c_epilogue.append( \
                            "jdouble tmp_%(n)s[%(cnt)i] = {%(args)s}; env->SetDoubleArrayRegion(%(n)s_out, 0, %(cnt)i, tmp_%(n)s);" %
                            {"n": a.name, "cnt": len(fields),
                             "args": ", ".join(["(jdouble)" + a.name + f[1] for f in fields])})
                        if type_dict[a.ctype]["j_type"] in ('bool', 'int', 'long', 'float', 'double'):
                            j_epilogue.append('if(%(n)s!=null) %(n)s[0] = (%(t)s)%(n)s_out[0];' % {'n': a.name, 't':
                                type_dict[a.ctype]["j_type"]})
                        else:
                            set_vals = []
                            i = 0
                            for f in fields:
                                set_vals.append("%(n)s%(f)s = %(t)s%(n)s_out[%(i)i]" %
                                                {"n": a.name,
                                                 "t": ("(" + type_dict[f[0]]["j_type"] + ")", "")[f[0] == "double"],
                                                 "f": f[1], "i": i}
                                                )
                                i += 1
                            j_epilogue.append("if(" + a.name + "!=null){ " + "; ".join(set_vals) + "; } ")

            # calculate java method signature to check for uniqueness
            j_args = []
            for a in args:
                if not a.ctype:  # hidden
                    continue
                jt = type_dict[a.ctype]["j_type"]
                if a.out and jt in ('bool', 'int', 'long', 'float', 'double'):
                    jt += '[]'
                j_args.append(jt + ' ' + a.name)
            j_signature = type_dict[fi.ctype]["j_type"] + " " + \
                          fi.jname + "(" + ", ".join(j_args) + ")"
            logging.info("java: " + j_signature)

            if (j_signature in j_signatures):
                if args:
                    args.pop()
                    continue
                else:
                    break

            # java part:
            # private java NATIVE method decl
            # e.g.
            # private static native void add_0(long src1, long src2, long dst, long mask, int dtype);
            jn_code.write(Template( \
                "    private static native $type $name($args);\n").substitute( \
                type=type_dict[fi.ctype].get("jn_type", "double[]"), \
                name=fi.jname + '_' + str(suffix_counter), \
                args=", ".join(["%s %s" % (type_dict[a.ctype]["jn_type"],
                                           a.name.replace(".", "_").replace("[", "").replace("]", "").replace(
                                               "_getNativeObjAddr()", "_nativeObj")) for a in jn_args])
            ));

            # java part:

            # java doc comment
            f_name = fi.name
            if fi.classname:
                f_name = fi.classname + "::" + fi.name
            java_doc = "//javadoc: " + f_name + "(%s)" % ", ".join([a.name for a in args if a.ctype])
            j_code.write(" " * 4 + java_doc + "\n")

            if fi.docstring:
                lines = StringIO(fi.docstring)
                for line in lines:
                    j_code.write(" " * 4 + line + "\n")
            if fi.annotation:
                j_code.write(" " * 4 + "\n".join(fi.annotation) + "\n")

            # public java wrapper method impl (calling native one above)
            # e.g.
            # public static void add( Mat src1, Mat src2, Mat dst, Mat mask, int dtype )
            # { add_0( src1.nativeObj, src2.nativeObj, dst.nativeObj, mask.nativeObj, dtype );  }
            ret_type = fi.ctype
            if fi.ctype.endswith('*'):
                ret_type = ret_type[:-1]
            ret_val = type_dict[ret_type]["j_type"] + " retVal = "
            tail = ""
            ret = "return retVal;"
            if "v_type" in type_dict[ret_type]:
                j_type = type_dict[ret_type]["j_type"]
                ret_val = "long[] addressList = "
                j_epilogue.append(j_type + ' retVal = Converters.Array_to_' + ret_type + '(addressList);')
            elif ret_type.startswith("Ptr_"):
                ret_val = type_dict[fi.ctype]["j_type"] + " retVal = " + type_dict[ret_type]["j_type"] + ".__fromPtr__("
                tail = ")"
            elif ret_type == "void":
                ret_val = ""
                ret = "return;"
            elif ret_type == "":  # c-tor
                if fi.classname and ci.base:
                    ret_val = "super( "
                    tail = " )"
                else:
                    ret_val = "nativeObj = "
                ret = "return;"
            elif self.isWrapped(camelCase(ret_type)):  # wrapped class
                ret_val = type_dict[ret_type]["j_type"] + " retVal = new " + camelCase(
                    self.getClass(camelCase(ret_type)).jname) + "("
                tail = ")"
            elif "jn_type" not in type_dict[ret_type]:
                ret_val = type_dict[fi.ctype]["j_type"] + " retVal = new " + camelCase(
                    type_dict[ret_type]["j_type"]) + "("
                tail = ")"

            static = "static"
            if fi.classname:
                static = fi.static

            j_code.write(Template( \
                """
    public $static $j_type $j_name($j_args)
    {
        $prologue
        $ret_val$jn_name($jn_args_call)$tail;
        $epilogue
        $ret
    }
                """
            ).substitute( \
                ret=ret, \
                ret_val=ret_val, \
                tail=tail, \
                prologue="\n        ".join(j_prologue), \
                epilogue="\n        ".join(j_epilogue), \
                static=static, \
                j_type=type_dict[fi.ctype]["j_type"], \
                j_name=fi.jname, \
                j_args=", ".join(j_args), \
                jn_name=fi.jname + '_' + str(suffix_counter), \
                jn_args_call=", ".join([a.name for a in jn_args]), \
                )
            )

            # cpp part:
            # jni_func(..) { _retval_ = vp_func(..); return _retval_; }
            ret = "return _retval_;"
            default = "return 0;"
            if fi.ctype == "void":
                ret = "return;"
                default = "return;"
            elif not fi.ctype:  # c-tor
                ret = "return (jlong) _retval_;"
            elif "v_type" in type_dict[fi.ctype]:  # c-tor
                if type_dict[fi.ctype]["v_type"] in ("vpMatrix", "vector_vpMatrix"):
                    ret = "return (jlong) _retval_;"
                else:  # returned as jobject
                    ret = "return _retval_;"
            elif fi.ctype == "String":
                ret = "return env->NewStringUTF(_retval_.c_str());"
                default = 'return env->NewStringUTF("");'
            elif self.isWrapped(camelCase(fi.ctype)):  # wrapped class:
                ret = "return (jlong) new %s(_retval_);" % self.smartWrap(ci, self.fullTypeName(camelCase(fi.ctype)))
            elif fi.ctype.startswith('Ptr_'):
                c_prologue.append("typedef Ptr<%s> %s;" % (self.fullTypeName(fi.ctype[4:]), fi.ctype))
                ret = "return (jlong)(new %(ctype)s(_retval_));" % {'ctype': fi.ctype}
            elif self.isWrapped(camelCase(ret_type)):  # pointer to wrapped class:
                ret = "return (jlong) _retval_;"
            elif type_dict[fi.ctype]["jni_type"] == "jdoubleArray":
                ret = "return _da_retval_;"

            # hack: replacing func call with property set/get
            name = fi.name
            if prop_name:
                if args:
                    name = prop_name + " = "
                else:
                    name = prop_name + ";//"

            cvname = fi.fullName(isCPP=True)
            retval = self.fullTypeName(fi.ctype) + " _retval_ = "
            if fi.ctype == "void":
                retval = ""
            elif fi.ctype == "String":
                retval = "vp::" + retval
            elif "v_type" in type_dict[fi.ctype]:  # vector is returned
                retval = type_dict[fi.ctype]['jni_var'] % {"n": '_ret_val_vector_'} + " = "
                c_epilogue.append("jlongArray _retval_ = " + fi.ctype + "_to_List(env, _ret_val_vector_);")
            if len(fi.classname) > 0:
                if not fi.ctype:  # c-tor
                    retval = reverseCamelCase(fi.fullClass(isCPP=True)) + "* _retval_ = "
                    cvname = "new " + reverseCamelCase(fi.fullClass(isCPP=True))
                elif fi.static:
                    cvname = reverseCamelCase(fi.fullName(isCPP=True))
                else:
                    cvname = ("me->" if not self.isSmartClass(ci) else "(*me)->") + name
                    c_prologue.append( \
                        "%(cls)s* me = (%(cls)s*) self; //TODO: check for NULL" \
                        % {"cls": reverseCamelCase(self.smartWrap(ci, fi.fullClass(isCPP=True)))} \
                        )
            cvargs = []
            for a in args:
                if a.pointer:
                    jni_name = "&%(n)s"
                else:
                    jni_name = "%(n)s"
                    if not a.out and not "jni_var" in type_dict[a.ctype]:
                        # explicit cast to C type to avoid ambiguous call error on platforms (mingw)
                        # where jni types are different from native types (e.g. jint is not the same as int)
                        jni_name = "(%s)%s" % (a.ctype, jni_name)
                if not a.ctype:  # hidden
                    jni_name = a.defval
                cvargs.append(type_dict[a.ctype].get("jni_name", jni_name) % {"n": a.name})
                if "v_type" not in type_dict[a.ctype]:
                    if ("I" in a.out or not a.out or self.isWrapped(a.ctype)) and "jni_var" in type_dict[a.ctype]:  # complex type
                        if a.ctype in [ 'vector_double', 'vector_float' ]:
                            c_prologue.append(type_dict[a.ctype]["jni_var"] % {"n": a.name} + "_ = List_to_" + a.ctype + "(env, " + a.name + ");")
                            # add "_" suffix to the last argument
                            cvargs[len(cvargs) - 1] = cvargs[len(cvargs) - 1] + "_"
                        else:
                            c_prologue.append(type_dict[a.ctype]["jni_var"] % {"n": a.name} + ";")
                    if a.out and "I" not in a.out and not self.isWrapped(a.ctype) and a.ctype:
                        c_prologue.append("%s %s;" % (a.ctype, a.name))

            # Add 3rd party specific tags
            # If Gsl or Lapack or OpenCV is missing, don't include them to prevent compilation error
            if fi.name in ['detByLUGsl',    'svdGsl',    'inverseByLUGsl',    'pseudoInverseGsl']:
                ret += '\n    #endif'

            if fi.name in ['detByLUOpenCV', 'svdOpenCV', 'inverseByLUOpenCV', 'pseudoInverseOpenCV', 'inverseByOpenCV', 'inverseByCholeskyOpenCV']:
                ret += '\n    #endif'

            if fi.name in ['detByLUEigen3', 'svdEigen3', 'inverseByLUEigen3', 'pseudoInverseEigen3', 'inverseByEigen3']:
                ret += '\n    #endif'

            if fi.name in ['detByLULapack', 'svdLapack', 'inverseByLULapack', 'pseudoInverseLapack', 'inverseByLapack', 'inverseByCholeskyLapack', 'inverseByQRLapack']:
                ret += '\n    #endif'

            rtype = type_dict[fi.ctype].get("jni_type", "jdoubleArray")
            clazz = ci.jname
            cpp_code.write(Template( \
                """
${namespace}
JNIEXPORT $rtype JNICALL Java_org_visp_${module}_${clazz}_$fname ($argst);

JNIEXPORT $rtype JNICALL Java_org_visp_${module}_${clazz}_$fname
  ($args)
{
  static const char method_name[] = "$module::$fname()";
  try {
    LOGD("%s", method_name);
    $prologue
    $retval$cvname( ${cvargs} );
    $epilogue$ret
  } catch(const std::exception &e) {
    throwJavaException(env, &e, method_name);
  } catch (...) {
    throwJavaException(env, 0, method_name);
  }
  $default
}
                """).substitute( \
                rtype=rtype, \
                module=self.module.replace('_', '_1'), \
                clazz=clazz.replace('_', '_1'), \
                fname=(fi.jname + '_' + str(suffix_counter)).replace('_', '_1'), \
                args=", ".join(["%s %s" % (type_dict[a.ctype].get("jni_type"), a.name) for a in jni_args]), \
                argst=", ".join([type_dict[a.ctype].get("jni_type") for a in jni_args]), \
                prologue="\n        ".join(c_prologue), \
                epilogue="  ".join(c_epilogue) + ("\n        " if c_epilogue else ""), \
                ret=ret, \
                cvname=cvname, \
                cvargs=", ".join(cvargs), \
                default=default, \
                retval=retval, \
                namespace=('using namespace ' + ci.namespace.replace('.', '::') + ';') if ci.namespace else ''
            ))

            # adding method signature to dictionarry
            j_signatures.append(j_signature)

            # processing args with default values
            if not args or not args[-1].defval:
                break
            while args and args[-1].defval:
                # 'smart' overloads filtering
                a = args.pop()
                if a.name in ('mask', 'dtype', 'ddepth', 'lineType', 'borderType', 'borderMode', 'criteria'):
                    break

    # INFO: Regex starts here
    def gen_class(self, ci):
        logging.info("%s", ci)
        # constants
        if ci.private_consts:
            logging.info("%s", ci.private_consts)
            ci.j_code.write("""
    private static final int
            %s;\n\n""" % (",\n" + " " * 12).join(["%s = %s" % (c.name, c.value) for c in ci.private_consts])
                            )
        if ci.consts:
            logging.info("%s", ci.consts)
            ci.j_code.write("""
    public static final int
            %s;\n\n""" % (",\n" + " " * 12).join(["%s = %s" % (c.name, c.value) for c in ci.consts])
                            )
        # methods
        for fi in ci.getAllMethods():
            self.gen_func(ci, fi)
        # props
        for pi in ci.props:
            # getter
            getter_name = ci.fullName() + ".get_" + pi.name
            fi = FuncInfo([getter_name, pi.ctype, [], []],
                          self.namespaces)  # [ funcname, return_ctype, [modifiers], [args] ]
            self.gen_func(ci, fi, pi.name)
            if pi.rw:
                # setter
                setter_name = ci.fullName() + ".set_" + pi.name
                fi = FuncInfo([setter_name, "void", [], [[pi.ctype, pi.name, "", [], ""]]], self.namespaces)
                self.gen_func(ci, fi, pi.name)

        # manual ports
        if ci.name in ManualFuncs:
            for func in ManualFuncs[ci.name].keys():
                ci.j_code.write("\n\t")
                ci.jn_code.write("\n\t")
                ci.cpp_code.write("\n")
                ci.j_code.write("\n\t".join(ManualFuncs[ci.name][func]["j_code"]))
                ci.jn_code.write("\n\t".join(ManualFuncs[ci.name][func]["jn_code"]))
                ci.cpp_code.write("\n".join(ManualFuncs[ci.name][func]["cpp_code"]))
                ci.j_code.write("\n\t")
                ci.jn_code.write("\n\t")
                ci.cpp_code.write("\n")

        # Add only classes that support << operator
        if ci.name in ToStringSupport:
            # toString
            ci.j_code.write(
                """
    @Override
    public String toString(){
        return toString(nativeObj);
    }
                """)

            ci.jn_code.write(
                """
    // native support for java toString()
    private static native String toString(long nativeObj);
                """)

            # native support for java toString()
            ci.cpp_code.write("""
//
//  native support for java toString()
//  static String %(cls)s::toString()
//

JNIEXPORT jstring JNICALL Java_org_visp_%(module)s_%(j_cls)s_toString(JNIEnv*, jclass, jlong);

JNIEXPORT jstring JNICALL Java_org_visp_%(module)s_%(j_cls)s_toString
  (JNIEnv* env, jclass, jlong self)
{
  %(cls)s* me = (%(cls)s*) self; //TODO: check for NULL
  std::stringstream ss;
  ss << *me;
  return env->NewStringUTF(ss.str().c_str());
}

                """ % {"module": module.replace('_', '_1'), "cls": self.smartWrap(ci, ci.fullName(isCPP=True)),
                       "j_cls": ci.jname.replace('_', '_1')}
            )

        if ci.name != 'VpImgproc' and ci.name != self.Module or ci.base:
            # finalize()
            ci.j_code.write(
                """
    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }
                """)

            ci.jn_code.write(
                """
    // native support for java finalize()
    private static native void delete(long nativeObj);
                """)

            # native support for java finalize()
            ci.cpp_code.write( \
                """
//
//  native support for java finalize()
//  static void %(cls)s::delete( __int64 self )
//

JNIEXPORT void JNICALL Java_org_visp_%(module)s_%(j_cls)s_delete(JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_%(module)s_%(j_cls)s_delete
  (JNIEnv*, jclass, jlong self)
{
  delete (%(cls)s*) self;
}

                """ % {"module": module.replace('_', '_1'), "cls": self.smartWrap(ci, ci.fullName(isCPP=True)),
       "j_cls": ci.jname.replace('_', '_1')}
            )

    def getClass(self, classname):
        return self.classes[classname or self.Module]

    def isWrapped(self, classname):
        name = classname or self.Module
        return name in self.classes

    def isSmartClass(self, ci):
        '''
        Check if class stores Ptr<T>* instead of T* in nativeObj field
        '''
        if ci.smart != None:
            return ci.smart

        # if parents are smart (we hope) then children are!
        # if not we believe the class is smart if it has "create" method
        ci.smart = False
        if ci.base or ci.name == 'Algorithm':
            ci.smart = True
        else:
            for fi in ci.methods:
                if fi.name == "create":
                    ci.smart = True
                    break

        return ci.smart

    def smartWrap(self, ci, fullname):
        '''
        Wraps fullname with Ptr<> if needed
        '''
        if self.isSmartClass(ci):
            return "Ptr<" + fullname + ">"
        if fullname[0] == ':':   # some classes miss namespace. so they are named ::<class>. That :: should be removed
            fullname = fullname[2:]
        return fullname

    '''
        # INFO: Generate a visp_jni.hpp file. Contains #include tags for all
        modules that very to be built and specified at compile time
    '''

    def finalize(self, output_jni_path):
        list_file = os.path.join(output_jni_path, "visp_jni.hpp")
        self.save(list_file, '\n'.join(['#include "%s"' % f for f in self.cpp_files]))


'''
    # INFO: As name suggests, copies some files specified at the desired location.
    Are the files edited as they are copied? No
'''


def copy_java_files(java_files_dir, java_base_path, default_package_path='org/visp/'):
    global total_files, updated_files
    java_files = []
    re_filter = re.compile(r'^.+\.(java|aidl)(.in)?$')
    for root, dirnames, filenames in os.walk(java_files_dir):
        java_files += [os.path.join(root, filename) for filename in filenames if re_filter.match(filename)]
    java_files = [f.replace('\\', '/') for f in java_files]

    re_package = re.compile(r'^package +(.+);')
    re_prefix = re.compile(r'^.+[\+/]([^\+]+).(java|aidl)(.in)?$')
    for java_file in java_files:

        '''
            # INFO: Not all files are copied directly. There's a set of files
            read in the `config.json`. Instead of copyong them, the code copies
            a diffrent set of files(also mentioned in gen_config.json, stored as
            a dict).
        '''

        src = checkFileRemap(java_file)
        with open(src, 'r') as f:
            package_line = f.readline()
        m = re_prefix.match(java_file)

        # INFO: using absolute path, get the filename.extension <- @a
        target_fname = (m.group(1) + '.' + m.group(2)) if m else os.path.basename(java_file)

        # INFO: Using the package name mentioned at the top of java file
        # INFO: generate a path. Ex package org.visp.core to ./org/visp/core <- @b
        m = re_package.match(package_line)
        if m:
            package = m.group(1)
            package_path = package.replace('.', '/')
        else:
            package_path = default_package_path

        # print(java_file, package_path, target_fname)

        # INFO: dest path = ./gen/java/@b/@a
        # INFO: ./gen/java is given as function argument
        dest = os.path.join(java_base_path, os.path.join(package_path, target_fname))
        assert dest[-3:] != '.in', dest + ' | ' + target_fname

        mkdir_p(os.path.dirname(dest))
        total_files += 1

        if (not os.path.exists(dest)) or (os.stat(src).st_mtime - os.stat(dest).st_mtime > 1):
            copyfile(src, dest)
            updated_files += 1


if __name__ == "__main__":
    # initialize logger
    logging.basicConfig(filename='gen_java.log', format=None, filemode='w', level=logging.INFO)
    handler = logging.StreamHandler()
    handler.setLevel(logging.WARNING)
    logging.getLogger().addHandler(handler)

    # parse command line parameters
    import argparse

    arg_parser = argparse.ArgumentParser(description='ViSP Java Wrapper Generator')
    arg_parser.add_argument('-p', '--parser', required=True, help='ViSP header parser')
    arg_parser.add_argument('-c', '--config', required=True, help='ViSP modules config')

    args = arg_parser.parse_args()

    # INFO: There's a file called gen2.py. It contains some functions Arginfo
    # , GeneralINfo, FuncIfno and some template strings. I guess its called
    # header parsers. I guess thats used for extracting Class name, function name
    # and module name from a .cpp file. These values must be fed in the
    # template given above to create JNI code.

    # import header parser
    hdr_parser_path = os.path.abspath(args.parser)
    if hdr_parser_path.endswith(".py"):
        hdr_parser_path = os.path.dirname(hdr_parser_path)
    sys.path.append(hdr_parser_path)
    import hdr_parser

    with open(args.config) as f:
        config = json.load(f)

    ROOT_DIR = config['rootdir'];
    assert os.path.exists(ROOT_DIR)
    FILES_REMAP = {os.path.realpath(os.path.join(ROOT_DIR, f['src'])): f['target'] for f in config['files_remap']}
    logging.info("\nRemapped configured files (%d):\n%s", len(FILES_REMAP), pformat(FILES_REMAP))

    # INFO: Now we create some folders inside the <build> folder viz. gen/cpp, gen/java

    dstdir = "./gen"
    jni_path = os.path.join(dstdir, 'cpp');
    mkdir_p(jni_path)
    java_base_path = os.path.join(dstdir, 'java');
    mkdir_p(java_base_path)
    java_test_base_path = os.path.join(dstdir, 'test');
    mkdir_p(java_test_base_path)

    for (subdir, target_subdir) in [('src/java', 'java'), ('android/java', None), ('android-21/java', None)]:
        if target_subdir is None:
            target_subdir = subdir
        java_files_dir = os.path.join(SCRIPT_DIR, subdir)
        if os.path.exists(java_files_dir):
            target_path = os.path.join(dstdir, target_subdir);
            mkdir_p(target_path)
            copy_java_files(java_files_dir, target_path)

    # launch Java Wrapper generator
    generator = JavaWrapperGenerator()

    gen_dict_files = []

    # INFO: I'm not adding modules manually. That info is read from <buildPath>/modules/java_bindings_generator/gen_java.json
    # To add Java Wrapper for a module, find and change the line given below in <module>/CMakeLists.txt:
    # vp_add_module(<mod-name> ....)   -->   vp_add_module(<mod-name> .... WRAP java)
    # Also you need to add support for functions that have to implemented manually in misc/java/<mod-name> folder
    print("JAVA: Processing ViSP modules: %d" % len(config['modules']))
    for e in config['modules']:

        # INFO: Get absolute location of the module
        (module, module_location) = (e['name'], os.path.join(ROOT_DIR, e['location']))
        logging.info("\n=== MODULE: %s (%s) ===\n" % (module, module_location))

        java_path = os.path.join(java_base_path, 'org/visp')
        mkdir_p(java_path)

        module_imports = []
        module_j_code = None
        module_jn_code = None
        srcfiles = []
        common_headers = []

        misc_location = os.path.join(hdr_parser_path, '../misc/' + module)

        # INFO: Get some specific .hpp files for the module. Not all modules contain the set of files
        srcfiles_fname = os.path.join(misc_location, 'filelist')
        if os.path.exists(srcfiles_fname):
            with open(srcfiles_fname) as f:
                srcfiles = [os.path.join(module_location, str(l).strip()) for l in f.readlines() if str(l).strip()]
        else:
            re_bad = re.compile(r'(private|.inl.hpp$|_inl.hpp$|.details.hpp$|_winrt.hpp$|/cuda/)')
            # .h files before .hpp
            h_files = []
            hpp_files = []
            for root, dirnames, filenames in os.walk(os.path.join(module_location, 'include')):
                h_files += [os.path.join(root, filename) for filename in fnmatch.filter(filenames, '*.h')]
                hpp_files += [os.path.join(root, filename) for filename in fnmatch.filter(filenames, '*.hpp')]
            srcfiles = h_files + hpp_files
            srcfiles = [f for f in srcfiles if not re_bad.search(f.replace('\\', '/'))]
        logging.info("\nFiles (%d):\n%s", len(srcfiles), pformat(srcfiles))

        # INFO: Again, get some more .hpp files. Not for all modules
        common_headers_fname = os.path.join(misc_location, 'filelist_common')
        if os.path.exists(common_headers_fname):
            with open(common_headers_fname) as f:
                common_headers = [os.path.join(module_location, str(l).strip()) for l in f.readlines() if
                                  str(l).strip()]
        logging.info("\nCommon headers (%d):\n%s", len(common_headers), pformat(common_headers))

        '''
            # INFO: Not all C++ files can be directly turned to Java/JNI files. Get all
            such files and classes here. Include classes/functions that are to be ignored,
            new classes to be added manually. Sometimes arguments are to be changed while the
            function can be used

            Such files exist for a few root/core modules only like core, imgproc, calib
        '''
        gendict_fname = os.path.join(misc_location, 'gen_dict.json')
        if os.path.exists(gendict_fname):
            with open(gendict_fname) as f:
                gen_type_dict = json.load(f)
            class_ignore_list += gen_type_dict.get("class_ignore_list", [])
            const_ignore_list += gen_type_dict.get("const_ignore_list", [])
            const_private_list += gen_type_dict.get("const_private_list", [])
            missing_consts.update(gen_type_dict.get("missing_consts", {}))
            type_dict.update(gen_type_dict.get("type_dict", {}))
            ManualFuncs.update(gen_type_dict.get("ManualFuncs", {}))
            ToStringSupport += gen_type_dict.get("ToStringSupport", [])
            func_arg_fix.update(gen_type_dict.get("func_arg_fix", {}))
            if 'module_j_code' in gen_type_dict:
                module_j_code = read_contents(
                    checkFileRemap(os.path.join(misc_location, gen_type_dict['module_j_code'])))
            if 'module_jn_code' in gen_type_dict:
                module_jn_code = read_contents(
                    checkFileRemap(os.path.join(misc_location, gen_type_dict['module_jn_code'])))
            module_imports += gen_type_dict.get("module_imports", [])

        '''
            # INFO: In light of above, copy the .java files that were manually created to dst folder
            Later machine will generate all other files
        '''
        java_files_dir = os.path.join(misc_location, 'src/java')
        if os.path.exists(java_files_dir):
            copy_java_files(java_files_dir, java_base_path, 'org/visp/' + module)

        java_test_files_dir = os.path.join(misc_location, 'test')
        if os.path.exists(java_test_files_dir):
            copy_java_files(java_test_files_dir, java_test_base_path, 'org/visp/test/' + module)

        # INFO: Here's the meat. Most important function of the whole file
        if len(srcfiles) > 0:
            generator.gen(srcfiles, module, dstdir, jni_path, java_path, common_headers)
        else:
            logging.info("No generated code for module: %s", module)

    '''
        # INFO: Generate a visp_jni.hpp file. Contains #include tags for all
        modules that very to be built and specified at compile time
    '''
    generator.finalize(jni_path)

    print('Generated files: %d (updated %d)' % (total_files, updated_files))
