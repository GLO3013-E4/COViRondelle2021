import Vuetify from "vuetify";
import { mount } from "@vue/test-utils";

import RealTrajectory from "@/components/trajectories/RealTrajectory.vue";

const vuetify = new Vuetify();
const wrapper = mount(RealTrajectory, { vuetify });

describe("When mounting real trajectory", () => {
  it("Should mount", () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
