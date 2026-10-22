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
}

} // namespace zylann::voxel::pg
