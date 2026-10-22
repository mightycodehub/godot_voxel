#include "../../../util/containers/std_vector.h"
#include "../../../util/math/sdf.h"
#include "../../../util/profiling.h"
#include "../node_type_db.h"
#include "util.h"
#include <cmath>

namespace zylann::voxel::pg {

void register_sdf_nodes(Span<NodeType> types) {
	using namespace math;

	{
		NodeType &t = types[VoxelGraphFunction::NODE_SDF_PLANE];
		t.name = "SdfPlane";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("y", 0.f, VoxelGraphFunction::AUTO_CONNECT_Y));
		t.inputs.push_back(NodeType::Port("height"));
		t.outputs.push_back(NodeType::Port("sdf"));
		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			do_binop(ctx, [](float a, float b) { return a - b; });
		};
		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval a = ctx.get_input(0);
			const Interval b = ctx.get_input(1);
			ctx.set_output(0, a - b);
		};
		t.shader_gen_func = [](ShaderGenContext &ctx) {
			ctx.add_format("{} = {} - {};\n", ctx.get_output_name(0), ctx.get_input_name(0), ctx.get_input_name(1));
		};
	}
	{
		struct Params {
			float size_x;
			float size_y;
			float size_z;
		};
		NodeType &t = types[VoxelGraphFunction::NODE_SDF_BOX];
		t.name = "SdfBox";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("x", 0.f, VoxelGraphFunction::AUTO_CONNECT_X));
		t.inputs.push_back(NodeType::Port("y", 0.f, VoxelGraphFunction::AUTO_CONNECT_Y));
		t.inputs.push_back(NodeType::Port("z", 0.f, VoxelGraphFunction::AUTO_CONNECT_Z));
		t.params.push_back(NodeType::Param("size_x", Variant::FLOAT, 10.0));
		t.params.push_back(NodeType::Param("size_y", Variant::FLOAT, 10.0));
		t.params.push_back(NodeType::Param("size_z", Variant::FLOAT, 10.0));
		t.outputs.push_back(NodeType::Port("sdf"));
		t.compile_func = [](CompileContext &ctx) {
			Params p;
			p.size_x = ctx.get_param(0);
			p.size_y = ctx.get_param(1);
			p.size_z = ctx.get_param(2);
			ctx.set_params(p);
		};
		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			const Runtime::Buffer &x = ctx.get_input(0);
			const Runtime::Buffer &y = ctx.get_input(1);
			const Runtime::Buffer &z = ctx.get_input(2);
			const Params p = ctx.get_params<Params>();
			Runtime::Buffer &out = ctx.get_output(0);
			const Vector3f size(p.size_x, p.size_y, p.size_z);
			for (uint32_t i = 0; i < out.size; ++i) {
				out.data[i] = math::sdf_box(Vector3f(x.data[i], y.data[i], z.data[i]), size);
			}
		};
		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval x = ctx.get_input(0);
			const Interval y = ctx.get_input(1);
			const Interval z = ctx.get_input(2);
			const Params p = ctx.get_params<Params>();
			const Interval sx = Interval::from_single_value(p.size_x);
			const Interval sy = Interval::from_single_value(p.size_y);
			const Interval sz = Interval::from_single_value(p.size_z);
			ctx.set_output(0, math::sdf_box(x, y, z, sx, sy, sz));
		};
		t.shader_gen_func = [](ShaderGenContext &ctx) {
			ctx.require_lib_code(
					"sdf_box",
					"float vg_sdf_box(vec3 p, vec3 b) {\n"
					"	vec3 q = abs(p) - b;\n"
					"	return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);\n"
					"}\n"
			);
			ctx.add_format(
					"{} = vg_sdf_box(vec3({}, {}, {}), vec3({}, {}, {}));\n",
					ctx.get_output_name(0),
					ctx.get_input_name(0),
					ctx.get_input_name(1),
					ctx.get_input_name(2),
					float(ctx.get_param(0)),
					float(ctx.get_param(1)),
					float(ctx.get_param(2))
			);
		};
	}
	{
		NodeType &t = types[VoxelGraphFunction::NODE_SDF_SPHERE];
		t.name = "SdfSphere";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("x", 0.f, VoxelGraphFunction::AUTO_CONNECT_X));
		t.inputs.push_back(NodeType::Port("y", 0.f, VoxelGraphFunction::AUTO_CONNECT_Y));
		t.inputs.push_back(NodeType::Port("z", 0.f, VoxelGraphFunction::AUTO_CONNECT_Z));
		// Having radius as param allows to connect it to a singe Constant node, which can also be used elsewhere
		t.inputs.push_back(NodeType::Port("radius", 1.f, VoxelGraphFunction::AUTO_CONNECT_NONE, false));
		t.outputs.push_back(NodeType::Port("sdf"));
		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			const Runtime::Buffer &x = ctx.get_input(0);
			const Runtime::Buffer &y = ctx.get_input(1);
			const Runtime::Buffer &z = ctx.get_input(2);
			const Runtime::Buffer &r = ctx.get_input(3);
			Runtime::Buffer &out = ctx.get_output(0);
			// Squares of large coordinates exceed float precision, so the distance is computed in double.
			if (r.is_constant) {
				const double radius = r.constant_value;
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] =
							Math::sqrt(
									squared(double(x.data[i])) + squared(double(y.data[i])) + squared(double(z.data[i]))
							) -
							radius;
				}
			} else {
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] =
							Math::sqrt(
									squared(double(x.data[i])) + squared(double(y.data[i])) + squared(double(z.data[i]))
							) -
							double(r.data[i]);
				}
			}
		};
		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval x = ctx.get_input(0);
			const Interval y = ctx.get_input(1);
			const Interval z = ctx.get_input(2);
			const Interval r = ctx.get_input(3);
			const IntervalT<double> d = get_length(
												IntervalT<double>(x.min, x.max),
												IntervalT<double>(y.min, y.max),
												IntervalT<double>(z.min, z.max)
										) -
					IntervalT<double>(r.min, r.max);
			ctx.set_output(0, Interval(d.min, d.max));
		};
		t.shader_gen_func = [](ShaderGenContext &ctx) {
			ctx.add_format(
					"{} = length(vec3({}, {}, {})) - {};\n",
					ctx.get_output_name(0),
					ctx.get_input_name(0),
					ctx.get_input_name(1),
					ctx.get_input_name(2),
					ctx.get_input_name(3)
			);
		};
	}
	{
		struct Params {
			float r1;
			float r2;
		};
		NodeType &t = types[VoxelGraphFunction::NODE_SDF_TORUS];
		t.name = "SdfTorus";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("x", 0.f, VoxelGraphFunction::AUTO_CONNECT_X));
		t.inputs.push_back(NodeType::Port("y", 0.f, VoxelGraphFunction::AUTO_CONNECT_Y));
		t.inputs.push_back(NodeType::Port("z", 0.f, VoxelGraphFunction::AUTO_CONNECT_Z));
		t.outputs.push_back(NodeType::Port("sdf"));
		t.params.push_back(NodeType::Param("radius1", Variant::FLOAT, 16.f));
		t.params.push_back(NodeType::Param("radius2", Variant::FLOAT, 4.f));
		t.compile_func = [](CompileContext &ctx) {
			Params p;
			p.r1 = ctx.get_param(0);
			p.r2 = ctx.get_param(1);
			ctx.set_params(p);
		};
		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			const Runtime::Buffer &x = ctx.get_input(0);
			const Runtime::Buffer &y = ctx.get_input(1);
			const Runtime::Buffer &z = ctx.get_input(2);
			const Params p = ctx.get_params<Params>();
			Runtime::Buffer &out = ctx.get_output(0);
			for (uint32_t i = 0; i < out.size; ++i) {
				out.data[i] = math::sdf_torus(x.data[i], y.data[i], z.data[i], p.r1, p.r2);
			}
		};
		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval x = ctx.get_input(0);
			const Interval y = ctx.get_input(1);
			const Interval z = ctx.get_input(2);
			const Params p = ctx.get_params<Params>();
			const Interval r1 = Interval::from_single_value(p.r1);
			const Interval r2 = Interval::from_single_value(p.r2);
			ctx.set_output(0, math::sdf_torus(x, y, z, r1, r2));
		};
		t.shader_gen_func = [](ShaderGenContext &ctx) {
			ctx.require_lib_code(
					"sdf_torus",
					"float vg_sdf_torus(vec3 p, vec2 t) {\n"
					"	vec2 q = vec2(length(p.xz) - t.x, p.y);\n"
					"	return length(q) - t.y;\n"
					"}\n"
			);
			ctx.add_format(
					"{} = vg_sdf_torus(vec3({}, {}, {}), vec2({}, {}));\n",
					ctx.get_output_name(0),
					ctx.get_input_name(0),
					ctx.get_input_name(1),
					ctx.get_input_name(2),
					float(ctx.get_param(0)),
					float(ctx.get_param(1))
			);
		};
	}
	{
		struct Params {
			float smoothness;
		};
		NodeType &t = types[VoxelGraphFunction::NODE_SDF_SMOOTH_UNION];
		t.name = "SdfSmoothUnion";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("a"));
		t.inputs.push_back(NodeType::Port("b"));
		t.outputs.push_back(NodeType::Port("sdf"));
		t.params.push_back(NodeType::Param("smoothness", Variant::FLOAT, 0.f));
		t.compile_func = [](CompileContext &ctx) {
			Params p;
			p.smoothness = ctx.get_param(0).operator float();
			ctx.set_params(p);
		};
		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			ZN_PROFILE_SCOPE_NAMED("NODE_SDF_SMOOTH_UNION");
			bool a_ignored;
			bool b_ignored;
			const Runtime::Buffer &a = ctx.try_get_input(0, a_ignored);
			const Runtime::Buffer &b = ctx.try_get_input(1, b_ignored);
			Runtime::Buffer &out = ctx.get_output(0);
			const Params params = ctx.get_params<Params>();
			if (a_ignored) {
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = b.data[i];
				}
			} else if (b_ignored) {
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = a.data[i];
				}
			} else if (params.smoothness > 0.0001f) {
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = math::sdf_smooth_union(a.data[i], b.data[i], params.smoothness);
				}
			} else {
				// Fallback on hard-union, smooth union does not support zero smoothness
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = math::sdf_union(a.data[i], b.data[i]);
				}
			}
		};
		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval a = ctx.get_input(0);
			const Interval b = ctx.get_input(1);
			const Params params = ctx.get_params<Params>();

			if (params.smoothness > 0.0001f) {
				// TODO Ideally we should be consistent in which kind of floats we use.
				// Right now we should use `float` everywhere, eventually allowing to choose even if Godot is compiled
				// with doubles, as not everything actually needs to be double
				const math::SdfAffectingArguments args =
						math::sdf_polynomial_smooth_union_side<real_t>(a, b, params.smoothness);
				switch (args) {
					case math::SDF_ONLY_A:
						ctx.ignore_input(1);
						break;
					case math::SDF_ONLY_B:
						ctx.ignore_input(0);
						break;
					case math::SDF_BOTH:
						break;
					default:
						CRASH_NOW();
						break;
				}
				ctx.set_output(0, math::sdf_smooth_union<real_t>(a, b, params.smoothness));

			} else {
				const math::SdfAffectingArguments args = math::sdf_union_side(a, b);
				switch (args) {
					case math::SDF_ONLY_A:
						ctx.ignore_input(1);
						break;
					case math::SDF_ONLY_B:
						ctx.ignore_input(0);
						break;
					case math::SDF_BOTH:
						break;
					default:
						CRASH_NOW();
						break;
				}
				ctx.set_output(0, math::sdf_union(a, b));
			}
		};
		t.shader_gen_func = [](ShaderGenContext &ctx) {
			ctx.require_lib_code(
					"sdf_smooth_union",
					"float vg_sdf_smooth_union(float a, float b, float s) {\n"
					"	float h = clamp(0.5 + 0.5 * (b - a) / s, 0.0, 1.0);\n"
					"	return mix(b, a, h) - s * h * (1.0 - h);\n"
					"}\n"
			);
			ctx.add_format(
					"{} = vg_sdf_smooth_union({}, {}, {});\n",
					ctx.get_output_name(0),
					ctx.get_input_name(0),
					ctx.get_input_name(1),
					float(ctx.get_param(0))
			);
		};
	}
	{
		struct Params {
			float smoothness;
		};
		NodeType &t = types[VoxelGraphFunction::NODE_SDF_SMOOTH_SUBTRACT];
		t.name = "SdfSmoothSubtract";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("a"));
		t.inputs.push_back(NodeType::Port("b"));
		t.outputs.push_back(NodeType::Port("sdf"));
		t.params.push_back(NodeType::Param("smoothness", Variant::FLOAT, 0.f));
		t.compile_func = [](CompileContext &ctx) {
			Params p;
			p.smoothness = ctx.get_param(0).operator float();
			ctx.set_params(p);
		};
		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			ZN_PROFILE_SCOPE_NAMED("NODE_SDF_SMOOTH_SUBTRACT");
			bool a_ignored;
			bool b_ignored;
			const Runtime::Buffer &a = ctx.try_get_input(0, a_ignored);
			const Runtime::Buffer &b = ctx.try_get_input(1, b_ignored);
			Runtime::Buffer &out = ctx.get_output(0);
			const Params params = ctx.get_params<Params>();
			if (a_ignored) {
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = -b.data[i];
				}
			} else if (b_ignored) {
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = a.data[i];
				}
			} else if (params.smoothness > 0.0001f) {
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = math::sdf_smooth_subtract(a.data[i], b.data[i], params.smoothness);
				}
			} else {
				// Fallback on hard-subtract, smooth subtract does not support zero smoothness
				for (uint32_t i = 0; i < out.size; ++i) {
					out.data[i] = math::sdf_subtract(a.data[i], b.data[i]);
				}
			}
		};
		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval a = ctx.get_input(0);
			const Interval b = ctx.get_input(1);
			const Params params = ctx.get_params<Params>();

			if (params.smoothness > 0.0001f) {
				const math::SdfAffectingArguments args =
						math::sdf_polynomial_smooth_subtract_side<real_t>(a, b, params.smoothness);
				switch (args) {
					case math::SDF_ONLY_A:
						ctx.ignore_input(1);
						break;
					case math::SDF_ONLY_B:
						ctx.ignore_input(0);
						break;
					case math::SDF_BOTH:
						break;
					default:
						CRASH_NOW();
						break;
				}
				ctx.set_output(0, math::sdf_smooth_subtract<real_t>(a, b, params.smoothness));

			} else {
				const math::SdfAffectingArguments args = math::sdf_subtract_side(a, b);
				switch (args) {
					case math::SDF_ONLY_A:
						ctx.ignore_input(1);
						break;
					case math::SDF_ONLY_B:
						ctx.ignore_input(0);
						break;
					case math::SDF_BOTH:
						break;
					default:
						CRASH_NOW();
						break;
				}
				ctx.set_output(0, math::sdf_subtract(a, b));
			}
		};
		t.shader_gen_func = [](ShaderGenContext &ctx) {
			ctx.require_lib_code(
					"sdf_smooth_subtract",
					"float vg_sdf_smooth_subtract(float b, float a, float s) {\n"
					"	float h = clamp(0.5 - 0.5 * (b + a) / s, 0.0, 1.0);\n"
					"	return mix(b, -a, h) + s * h * (1.0 - h);\n"
					"}\n"
			);
			ctx.add_format(
					"{} = vg_sdf_smooth_subtract({}, {}, {});\n",
					ctx.get_output_name(0),
					ctx.get_input_name(0),
					ctx.get_input_name(1),
					float(ctx.get_param(0))
			);
		};
	}
	{
		NodeType &t = types[VoxelGraphFunction::NODE_SDF_PREVIEW];
		t.name = "SdfPreview";
		t.category = CATEGORY_DEBUG;
		t.inputs.push_back(NodeType::Port("value"));

		t.params.push_back(NodeType::Param("min_value", Variant::FLOAT, -1.f));
		t.params.push_back(NodeType::Param("max_value", Variant::FLOAT, 1.f));
		t.params.push_back(NodeType::Param("fraction_period", Variant::FLOAT, 10.f));

		// Matches an enum in editor code `VoxelGraphEditorNodePreview`
		NodeType::Param mode_param("mode", Variant::INT, 0);
		mode_param.enum_items.push_back("Greyscale");
		mode_param.enum_items.push_back("SDF");
		t.params.push_back(mode_param);

		t.debug_only = true;
		t.is_pseudo_node = true;
	}
	{
		struct Site {
			double cx;
			double cy;
			double cz;
			double flat;
			double rim;
			double height;
			double outer_sq;
		};
		struct SiteTable {
			StdVector<Site> sites;
		};
		struct Params {
			const SiteTable *table;
		};

		NodeType &t = types[VoxelGraphFunction::NODE_PLANET_SITES];
		t.name = "PlanetSites";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("x", 0.f, VoxelGraphFunction::AUTO_CONNECT_X));
		t.inputs.push_back(NodeType::Port("y", 0.f, VoxelGraphFunction::AUTO_CONNECT_Y));
		t.inputs.push_back(NodeType::Port("z", 0.f, VoxelGraphFunction::AUTO_CONNECT_Z));
		t.inputs.push_back(NodeType::Port("radius", 1.f, VoxelGraphFunction::AUTO_CONNECT_NONE, false));
		t.inputs.push_back(NodeType::Port("height"));
		t.outputs.push_back(NodeType::Port("out"));
		t.params.push_back(NodeType::Param("sites", Variant::PACKED_FLOAT64_ARRAY, PackedFloat64Array()));

		t.compile_func = [](CompileContext &ctx) {
			const PackedFloat64Array data = ctx.get_param(0);
			if (data.size() % 6 != 0) {
				ctx.make_error(ZN_TTR("The site table must hold six numbers per site"));
				return;
			}
			SiteTable *table = ZN_NEW(SiteTable);
			for (int64_t i = 0; i < data.size(); i += 6) {
				const double len = Math::sqrt(squared(data[i]) + squared(data[i + 1]) + squared(data[i + 2]));
				const double flat = data[i + 3];
				const double rim = data[i + 4];
				if (!(len > 0.0) || !(flat >= 0.0) || !(rim > 0.0)) {
					ZN_DELETE(table);
					ctx.make_error(
							String(ZN_TTR("Site {0} needs a direction, a pad radius of at least zero and a rim wider "
										  "than zero"))
									.format(varray(i / 6))
					);
					return;
				}
				table->sites.push_back(
						Site{ data[i] / len,
							  data[i + 1] / len,
							  data[i + 2] / len,
							  flat,
							  rim,
							  data[i + 5],
							  squared(flat + rim) }
				);
			}
			Params p;
			p.table = table;
			ctx.set_params(p);
			ctx.add_delete_cleanup(table);
		};

		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			ZN_PROFILE_SCOPE_NAMED("NODE_PLANET_SITES");
			const Runtime::Buffer &x = ctx.get_input(0);
			const Runtime::Buffer &y = ctx.get_input(1);
			const Runtime::Buffer &z = ctx.get_input(2);
			const Runtime::Buffer &r = ctx.get_input(3);
			const Runtime::Buffer &h = ctx.get_input(4);
			Runtime::Buffer &out = ctx.get_output(0);
			const Params p = ctx.get_params<Params>();
			for (uint32_t i = 0; i < out.size; ++i) {
				const double px = x.data[i];
				const double py = y.data[i];
				const double pz = z.data[i];
				const double radius = r.is_constant ? r.constant_value : r.data[i];
				const double natural = h.data[i];
				const double len_sq = px * px + py * py + pz * pz;
				double ground = natural;
				for (const Site &s : p.table->sites) {
					if (px * s.cx + py * s.cy + pz * s.cz <= 0.0) {
						continue;
					}
					const double ax = py * s.cz - pz * s.cy;
					const double ay = pz * s.cx - px * s.cz;
					const double az = px * s.cy - py * s.cx;
					// Distance from the site's axis, scaled to the sphere: R sin θ, exact to nanometres.
					const double d_sq = radius * radius * (ax * ax + ay * ay + az * az) / len_sq;
					if (d_sq >= s.outer_sq) {
						continue;
					}
					const double t = math::clamp((Math::sqrt(d_sq) - s.flat) / s.rim, 0.0, 1.0);
					ground += (1.0 - t * t * (3.0 - 2.0 * t)) * (s.height - ground);
				}
				out.data[i] = ground - natural;
			}
		};

		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval x = ctx.get_input(0);
			const Interval y = ctx.get_input(1);
			const Interval z = ctx.get_input(2);
			const Interval r = ctx.get_input(3);
			const Interval h = ctx.get_input(4);
			const Params p = ctx.get_params<Params>();
			const double mx = 0.5 * (double(x.min) + double(x.max));
			const double my = 0.5 * (double(y.min) + double(y.max));
			const double mz = 0.5 * (double(z.min) + double(z.max));
			const double rho = 0.5 *
					Math::sqrt(squared(double(x.max) - x.min) + squared(double(y.max) - y.min) +
							   squared(double(z.max) - z.min));
			const double m_len = Math::sqrt(mx * mx + my * my + mz * mz);
			const double box_angle = m_len > rho ? Math::asin(rho / m_len) : Math::PI;
			double lo = 0.0;
			double hi = 0.0;
			for (const Site &s : p.table->sites) {
				const double reach = r.min > 0.f ? (Math::sqrt(s.outer_sq) + 1.0) / double(r.min) : 1.0;
				const double site_angle = reach < 1.0 ? Math::asin(reach) : Math::PI;
				const double ax = my * s.cz - mz * s.cy;
				const double ay = mz * s.cx - mx * s.cz;
				const double az = mx * s.cy - my * s.cx;
				const double angle =
						Math::atan2(Math::sqrt(ax * ax + ay * ay + az * az), mx * s.cx + my * s.cy + mz * s.cz);
				if (angle > box_angle + site_angle) {
					continue;
				}
				lo = math::min(lo, s.height - double(h.max));
				hi = math::max(hi, s.height - double(h.min));
			}
			// Rounded outward so the float estimate still encloses the double values.
			float lo_f = float(lo);
			float hi_f = float(hi);
			if (double(lo_f) > lo) {
				lo_f = std::nextafter(lo_f, -INFINITY);
			}
			if (double(hi_f) < hi) {
				hi_f = std::nextafter(hi_f, INFINITY);
			}
			ctx.set_output(0, Interval(lo_f, hi_f));
		};
	}
	{
		struct Road {
			double a[3];
			double b[3];
			double n[3];
			double t[3];
			double t_end;
			double half_width;
			double shoulder;
			double reach;
			double reach_sq;
			double reach_cos;
			double h_lo;
			double h_hi;
			StdVector<double> boundaries;
			StdVector<double> heights;
			StdVector<double> slopes;
		};
		struct SiteFade {
			double c[3];
			double area;
		};
		struct RoadTable {
			StdVector<Road> roads;
			StdVector<SiteFade> sites;
		};
		struct Params {
			const RoadTable *table;
		};

		NodeType &t = types[VoxelGraphFunction::NODE_PLANET_ROADS];
		t.name = "PlanetRoads";
		t.category = CATEGORY_SDF;
		t.inputs.push_back(NodeType::Port("x", 0.f, VoxelGraphFunction::AUTO_CONNECT_X));
		t.inputs.push_back(NodeType::Port("y", 0.f, VoxelGraphFunction::AUTO_CONNECT_Y));
		t.inputs.push_back(NodeType::Port("z", 0.f, VoxelGraphFunction::AUTO_CONNECT_Z));
		t.inputs.push_back(NodeType::Port("radius", 1.f, VoxelGraphFunction::AUTO_CONNECT_NONE, false));
		t.inputs.push_back(NodeType::Port("height"));
		t.outputs.push_back(NodeType::Port("out"));
		t.params.push_back(NodeType::Param("roads", Variant::PACKED_FLOAT64_ARRAY, PackedFloat64Array()));
		t.params.push_back(NodeType::Param("sites", Variant::PACKED_FLOAT64_ARRAY, PackedFloat64Array()));

		t.compile_func = [](CompileContext &ctx) {
			const PackedFloat64Array data = ctx.get_param(0);
			const PackedFloat64Array sites = ctx.get_param(1);
			RoadTable *table = ZN_NEW(RoadTable);
			const auto fail = [&ctx, table](const char *kind, int64_t index, const char *problem) {
				ZN_DELETE(table);
				ctx.make_error(String("PlanetRoads: {0} {1}: {2}").format(varray(kind, index, problem)));
			};
			int64_t i = 0;
			for (int64_t index = 0; i < data.size(); ++index) {
				const double k_value = data[i];
				if (!std::isfinite(k_value) || k_value < 1.0 || k_value != Math::floor(k_value)) {
					fail("road", index, "the step count must be a whole number of at least 1");
					return;
				}
				if (k_value > double(data.size())) {
					fail("road", index, "the record does not fit the table");
					return;
				}
				const int64_t k_count = int64_t(k_value);
				const int64_t length = 11 + 2 * k_count;
				if (i + length > data.size()) {
					fail("road", index, "the record does not fit the table");
					return;
				}
				for (int64_t j = i; j < i + length; ++j) {
					if (!std::isfinite(data[j])) {
						fail("road", index, "every value must be finite");
						return;
					}
				}
				Road road;
				for (int axis = 0; axis < 3; ++axis) {
					road.a[axis] = data[i + 1 + axis];
					road.b[axis] = data[i + 4 + axis];
				}
				const double a_length = Math::sqrt(squared(road.a[0]) + squared(road.a[1]) + squared(road.a[2]));
				const double b_length = Math::sqrt(squared(road.b[0]) + squared(road.b[1]) + squared(road.b[2]));
				if (Math::abs(a_length - 1.0) > 1e-9 || Math::abs(b_length - 1.0) > 1e-9) {
					fail("road", index, "both directions must have length 1");
					return;
				}
				road.half_width = data[i + 7];
				road.shoulder = data[i + 8];
				if (!(road.half_width > 0.0) || !(road.shoulder > 0.0)) {
					fail("road", index, "the half width and the shoulder must be above zero");
					return;
				}
				const double cross[3] = { road.a[1] * road.b[2] - road.a[2] * road.b[1],
										  road.a[2] * road.b[0] - road.a[0] * road.b[2],
										  road.a[0] * road.b[1] - road.a[1] * road.b[0] };
				const double cross_length = Math::sqrt(squared(cross[0]) + squared(cross[1]) + squared(cross[2]));
				if (!(cross_length >= 1e-12)) {
					fail("road", index, "the two directions must not be coincident or opposite");
					return;
				}
				for (int axis = 0; axis < 3; ++axis) {
					road.n[axis] = cross[axis] / cross_length;
				}
				road.t[0] = road.n[1] * road.a[2] - road.n[2] * road.a[1];
				road.t[1] = road.n[2] * road.a[0] - road.n[0] * road.a[2];
				road.t[2] = road.n[0] * road.a[1] - road.n[1] * road.a[0];
				road.t_end =
						cross_length / (1.0 + road.a[0] * road.b[0] + road.a[1] * road.b[1] + road.a[2] * road.b[2]);
				for (int64_t j = 0; j <= k_count; ++j) {
					road.boundaries.push_back(data[i + 9 + j]);
					road.heights.push_back(data[i + 10 + k_count + j]);
				}
				bool ordered = road.boundaries[0] >= 0.0 && road.boundaries[k_count] <= road.t_end;
				for (int64_t j = 0; j < k_count; ++j) {
					ordered = ordered && road.boundaries[j] < road.boundaries[j + 1];
				}
				if (!ordered) {
					fail("road", index, "the step boundaries must increase from 0 to the end of the arc");
					return;
				}
				road.reach = road.half_width + road.shoulder;
				road.reach_sq = road.reach * road.reach;
				if (!(road.reach < 1.0)) {
					fail("road", index, "the road must reach less than a quarter of the sphere");
					return;
				}
				road.reach_cos = Math::sqrt(1.0 - road.reach_sq);
				// Monotone cubic slopes (Fritsch-Carlson with Brodlie's weights), so no step is overshot.
				road.slopes.resize(k_count + 1, 0.0);
				for (int64_t j = 1; j < k_count; ++j) {
					const double before = road.boundaries[j] - road.boundaries[j - 1];
					const double after = road.boundaries[j + 1] - road.boundaries[j];
					const double secant_before = (road.heights[j] - road.heights[j - 1]) / before;
					const double secant_after = (road.heights[j + 1] - road.heights[j]) / after;
					if (secant_before * secant_after <= 0.0) {
						continue;
					}
					const double w1 = 2.0 * after + before;
					const double w2 = after + 2.0 * before;
					road.slopes[j] = (w1 + w2) / (w1 / secant_before + w2 / secant_after);
				}
				road.h_lo = road.heights[0];
				road.h_hi = road.heights[0];
				for (const double height : road.heights) {
					road.h_lo = math::min(road.h_lo, height);
					road.h_hi = math::max(road.h_hi, height);
				}
				table->roads.push_back(road);
				i += length;
			}
			if (sites.size() % 6 != 0) {
				fail("site", sites.size() / 6, "the site table must hold six numbers per site");
				return;
			}
			for (int64_t j = 0; j < sites.size(); j += 6) {
				const double len = Math::sqrt(squared(sites[j]) + squared(sites[j + 1]) + squared(sites[j + 2]));
				if (!(len > 0.0) || !(sites[j + 3] >= 0.0) || !(sites[j + 4] > 0.0)) {
					fail("site",
						 j / 6,
						 "a site needs a direction, a pad radius of at least zero and a rim wider than zero");
					return;
				}
				table->sites.push_back(
						SiteFade{ { sites[j] / len, sites[j + 1] / len, sites[j + 2] / len },
								  sites[j + 3] + sites[j + 4] }
				);
			}
			Params p;
			p.table = table;
			ctx.set_params(p);
			ctx.add_delete_cleanup(table);
		};

		t.process_buffer_func = [](Runtime::ProcessBufferContext &ctx) {
			ZN_PROFILE_SCOPE_NAMED("NODE_PLANET_ROADS");
			const Runtime::Buffer &x = ctx.get_input(0);
			const Runtime::Buffer &y = ctx.get_input(1);
			const Runtime::Buffer &z = ctx.get_input(2);
			const Runtime::Buffer &r = ctx.get_input(3);
			const Runtime::Buffer &h = ctx.get_input(4);
			Runtime::Buffer &out = ctx.get_output(0);
			const Params p = ctx.get_params<Params>();
			const RoadTable &table = *p.table;
			StdVector<double> alpha(table.sites.size());
			const auto set_alpha = [&table, &alpha](double radius) {
				for (size_t s = 0; s < table.sites.size(); ++s) {
					alpha[s] = table.sites[s].area / radius;
				}
			};
			if (r.is_constant) {
				set_alpha(r.constant_value);
			}
			for (uint32_t i = 0; i < out.size; ++i) {
				const double px = x.data[i];
				const double py = y.data[i];
				const double pz = z.data[i];
				const double len_sq = px * px + py * py + pz * pz;
				if (len_sq == 0.0) {
					out.data[i] = 0.f;
					continue;
				}
				if (!r.is_constant) {
					set_alpha(r.data[i]);
				}
				double len = 0.0;
				double weight_sum = 0.0;
				double weighted_height_sum = 0.0;
				for (const Road &road : table.roads) {
					const double q = px * road.n[0] + py * road.n[1] + pz * road.n[2];
					if (q * q >= road.reach_sq * len_sq) {
						continue;
					}
					if (len == 0.0) {
						len = Math::sqrt(len_sq);
					}
					const double along_a = px * road.a[0] + py * road.a[1] + pz * road.a[2];
					const double along_t = px * road.t[0] + py * road.t[1] + pz * road.t[2];
					const double denominator = Math::sqrt(along_a * along_a + along_t * along_t) + along_a;
					const double tau = denominator > 0.0 ? along_t / denominator : -1.0;
					double delta;
					double tau_c;
					if (denominator > 0.0 && tau >= 0.0 && tau <= road.t_end) {
						delta = Math::abs(q) / len;
						tau_c = tau;
					} else {
						double delta_a = INFINITY;
						if (along_a > 0.0) {
							delta_a = Math::sqrt(
											  squared(py * road.a[2] - pz * road.a[1]) +
											  squared(pz * road.a[0] - px * road.a[2]) +
											  squared(px * road.a[1] - py * road.a[0])
									  ) /
									len;
						}
						double delta_b = INFINITY;
						if (px * road.b[0] + py * road.b[1] + pz * road.b[2] > 0.0) {
							delta_b = Math::sqrt(
											  squared(py * road.b[2] - pz * road.b[1]) +
											  squared(pz * road.b[0] - px * road.b[2]) +
											  squared(px * road.b[1] - py * road.b[0])
									  ) /
									len;
						}
						delta = delta_a <= delta_b ? delta_a : delta_b;
						tau_c = delta_a <= delta_b ? 0.0 : road.t_end;
					}
					if (delta >= road.reach) {
						continue;
					}
					double weight = 1.0;
					if (delta > road.half_width) {
						const double u = (delta - road.half_width) / road.shoulder;
						weight = 1.0 - u * u * (3.0 - 2.0 * u);
					}
					for (size_t s = 0; s < table.sites.size() && weight > 0.0; ++s) {
						const SiteFade &site = table.sites[s];
						if (px * site.c[0] + py * site.c[1] + pz * site.c[2] <= 0.0) {
							continue;
						}
						const double delta_site = Math::sqrt(
														  squared(py * site.c[2] - pz * site.c[1]) +
														  squared(pz * site.c[0] - px * site.c[2]) +
														  squared(px * site.c[1] - py * site.c[0])
												  ) /
								len;
						if (delta_site <= alpha[s]) {
							weight = 0.0;
						} else if (delta_site < alpha[s] + road.shoulder) {
							const double u = (delta_site - alpha[s]) / road.shoulder;
							weight *= u * u * (3.0 - 2.0 * u);
						}
					}
					if (weight <= 0.0) {
						continue;
					}
					const size_t k_count = road.boundaries.size() - 1;
					double height;
					if (tau_c <= road.boundaries[0]) {
						height = road.heights[0];
					} else if (tau_c >= road.boundaries[k_count]) {
						height = road.heights[k_count];
					} else {
						size_t low = 0;
						size_t high = k_count;
						while (high - low > 1) {
							const size_t middle = (low + high) / 2;
							if (road.boundaries[middle] <= tau_c) {
								low = middle;
							} else {
								high = middle;
							}
						}
						const double d = road.boundaries[low + 1] - road.boundaries[low];
						const double u = (tau_c - road.boundaries[low]) / d;
						const double u2 = u * u;
						const double u3 = u2 * u;
						height = (2.0 * u3 - 3.0 * u2 + 1.0) * road.heights[low] +
								(u3 - 2.0 * u2 + u) * d * road.slopes[low] +
								(-2.0 * u3 + 3.0 * u2) * road.heights[low + 1] + (u3 - u2) * d * road.slopes[low + 1];
					}
					weight_sum += weight;
					weighted_height_sum += weight * height;
				}
				const double ground = h.data[i];
				out.data[i] = float((weighted_height_sum - ground * weight_sum) / math::max(1.0, weight_sum));
			}
		};

		t.range_analysis_func = [](Runtime::RangeAnalysisContext &ctx) {
			const Interval x = ctx.get_input(0);
			const Interval y = ctx.get_input(1);
			const Interval z = ctx.get_input(2);
			const Interval h = ctx.get_input(4);
			const Params p = ctx.get_params<Params>();
			const double cx = 0.5 * (double(x.min) + double(x.max));
			const double cy = 0.5 * (double(y.min) + double(y.max));
			const double cz = 0.5 * (double(z.min) + double(z.max));
			const double rho = 0.5 *
					Math::sqrt(squared(double(x.max) - x.min) + squared(double(y.max) - y.min) +
							   squared(double(z.max) - z.min));
			const double c_len = Math::sqrt(cx * cx + cy * cy + cz * cz);
			const double sin_a = c_len > rho ? rho / c_len : 1.0;
			const double cos_a = Math::sqrt(1.0 - sin_a * sin_a);
			bool counted = false;
			double h_lo = 0.0;
			double h_hi = 0.0;
			for (const Road &road : p.table->roads) {
				if (c_len > rho) {
					const double cos_sum = cos_a * road.reach_cos - sin_a * road.reach;
					if (cos_sum > 0.0) {
						const double sin_sum = sin_a * road.reach_cos + cos_a * road.reach;
						const double q = cx * road.n[0] + cy * road.n[1] + cz * road.n[2];
						const double along_a = cx * road.a[0] + cy * road.a[1] + cz * road.a[2];
						const double along_t = cx * road.t[0] + cy * road.t[1] + cz * road.t[2];
						const double along_b = cx * road.b[0] + cy * road.b[1] + cz * road.b[2];
						const double denominator = Math::sqrt(along_a * along_a + along_t * along_t) + along_a;
						const double tau = denominator > 0.0 ? along_t / denominator : -1.0;
						double delta;
						if (denominator > 0.0 && tau >= 0.0 && tau <= road.t_end) {
							delta = Math::abs(q) / c_len;
						} else if (along_a <= 0.0 && along_b <= 0.0) {
							continue;
						} else {
							delta = INFINITY;
							if (along_a > 0.0) {
								delta = Math::sqrt(
												squared(cy * road.a[2] - cz * road.a[1]) +
												squared(cz * road.a[0] - cx * road.a[2]) +
												squared(cx * road.a[1] - cy * road.a[0])
										) /
										c_len;
							}
							if (along_b > 0.0) {
								delta = math::min(
										delta,
										Math::sqrt(
												squared(cy * road.b[2] - cz * road.b[1]) +
												squared(cz * road.b[0] - cx * road.b[2]) +
												squared(cx * road.b[1] - cy * road.b[0])
										) / c_len
								);
							}
						}
						if (delta >= sin_sum + 1e-12) {
							continue;
						}
					}
				}
				h_lo = counted ? math::min(h_lo, road.h_lo) : road.h_lo;
				h_hi = counted ? math::max(h_hi, road.h_hi) : road.h_hi;
				counted = true;
			}
			if (!counted) {
				ctx.set_output(0, Interval::from_single_value(0.f));
				return;
			}
			const double lo = math::min(0.0, h_lo - double(h.max));
			const double hi = math::max(0.0, h_hi - double(h.min));
			// Widened by one float unit each way so the float estimate still encloses the double values.
			ctx.set_output(0, Interval(std::nextafter(float(lo), -INFINITY), std::nextafter(float(hi), INFINITY)));
		};
	}
}

} // namespace zylann::voxel::pg
