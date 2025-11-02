{{ objname | escape | underline}}

.. currentmodule:: {{ module }}

.. autoclass:: {{ objname }}
   :members:
   :show-inheritance:
   :member-order: groupwise
   :inherited-members: pybind11_builtins.pybind11_object, pybind11_object
   :special-members:

   {% if docstring %}
    {{ docstring }}
   {% endif %}

   {% block methods %}
   {% if methods %}
   .. rubric:: {{ _('Methods') }}

   .. autosummary::
      :nosignatures:
   {% for item in methods %}
      {%- if not item.startswith('_') and item not in inherited_members or item.startswith('__init__') %}
      ~{{ name }}.{{ item }}
      {%- endif -%}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block inheritedmethods %}
   {% if inherited_members %}
   .. rubric:: {{ _('Inherited Methods') }}

   .. autosummary::
      :nosignatures:
   {% for item in inherited_members %}
      {%- if not item.startswith('_') %}
      ~{{ name }}.{{ item }}
      {%- endif -%}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block operators %}
   {% if members %}
   .. rubric:: {{ _('Operators') }}

   .. autosummary::
      :nosignatures:
   {% for item in members %}
      {%- if item not in inherited_members and item.startswith('__') and item.endswith('__') %}
      ~{{ name }}.{{ item }}
      {%- endif -%}
   {%- endfor %}
   {% endif %}
   {% endblock %}

   {% block attributes %}
   {% if attributes %}
   .. rubric:: {{ _('Attributes') }}

   .. autosummary::
   {% for item in attributes %}
      ~{{ name }}.{{ item }}
   {%- endfor %}
   {% endif %}
   {% endblock %}
